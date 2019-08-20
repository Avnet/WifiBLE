/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This application forms part of the Wi-Fi setup and device control via BLE reference solution for 
// Azure Sphere using the Avnet Sphere MT3620 Starter Kit.
//
// It implements communication between an Azure Sphere MCU and the sibling application
// running on a Nordic nRF52 Bluetooth LE board, allowing Wi-Fi configuration and LED control on the
// Azure Sphere via Bluetooth LE.
//
// Pressing SK_BUTTON_A briefly will start allowing new BLE bonds for 1 minute.
// Holding SK_BUTTON_A for > 3 seconds will delete all BLE bonds.
// Pressing SKMPLE_BUTTON_B briefly will toggle SAMPLE_LED.
// Holding SK_BUTTON_B will forget all stored Wi-Fi networks on Azure Sphere.
// SK_USRLED (which is an tri-colored RGB LED) will illuminat/flash a color to indicates the BLE status:
//    Yellow/constant on - Uninitialized;
//    Blue/slow-blink    - Advertising to bonded devices only;
//    Red/slow-blink     - Advertising to all devices;
//    Green/slow-blink   - Connected to a BLE device;
//    Magenta/fast-blink - Error
//    White/constant on  - User LED on (locally initiated)
//    White/slow-blink   - User LED on (remote initiated)
//
// It uses the API for the following Azure Sphere application libraries:
// - UART (serial port)
// - GPIO (digital inputs and outputs)
// - log (messages shown in Visual Studio's Device Output window during debugging)
// - wificonfig (configure Wi-Fi settings)

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/gpio.h>
#include <applibs/uart.h>
#include <applibs/log.h>
#include <applibs/wificonfig.h>

// By default, this sample is targeted at the MT3620 Reference Development Board (RDB).
// This can be changed using the project property "Target Hardware Definition Directory".
// This #include imports the sample_hardware abstraction from that hardware definition.
#include <hw/sample_hardware.h>

// This sample uses a single-thread event loop pattern, based on epoll and timerfd
#include "epoll_timerfd_utilities.h"

#include "message_protocol.h"
#include "blecontrol_message_protocol.h"
#include "wificonfig_message_protocol.h"
#include "devicecontrol_message_protocol.h"


// define the RGB LED colors that will be used
// YELLOW  - Uninitialized;
// BLUE    - Advertising to bonded devices only;
// Red     - Advertising to all devices;
// Green   - Connected to a central device;
// Magenta - Error
//
typedef enum {
    BLACK=0,   /// RED=0, GREEN=0, BLUE=0
    BLUE,      /// RED=0, GREEN=0, BLUE=1
    GREEN,     /// RED=0, GREEN=1, BLUE=0
    CYAN,      /// RED=0, GREEN=1, BLUE=1
    RED,       /// RED=1, GREEN=0, BLUE=0
    MAGENTA,   /// RED=1, GREEN=0, BLUE=1
    YELLOW,    /// RED=1, GREEN=1, BLUE=0
    WHITE      /// RED=1, GREEN=1, BLUE=1
    } LedColor;

typedef enum { 
    LED_OFF=0,    /// Do not display any LED
    LED_BSLOW,    /// Blink the LED slowly (1 second period)
    LED_BFAST,    /// Blink the LED fast   (1/2 second period)
    LED_ON        /// LED ON
    } LedFlash;

// File descriptors - initialized to invalid value
static int buttonTimerFd = -1;
static int ledTimerFd = -1;
static int RedLedGpioFd = -1;
static int GreenLedGpioFd = -1;
static int BlueLedGpioFd = -1;
//jmf static int bleAdvertiseToBondedDevicesLedGpioFd = -1;
//jmf static int bleAdvertiseToAllDevicesLedGpioFd = -1;
//jmf static int bleConnectedLedGpioFd = -1;
//jmf static int deviceControlLedGpioFd = -1;
static int epollFd = -1;
static int uartFd = -1;
static int bleDeviceResetPinGpioFd = -1;
static struct timespec bleAdvertiseToAllTimeoutPeriod = {60u, 0};
static GPIO_Value_Type deviceControlLedState = GPIO_Value_High;

/// <summary>
///     Button events.
/// </summary>
typedef enum {
    ButtonEvent_Error = -1,       /// <summary>The event when failing to get button state.</summary>
    ButtonEvent_None = 0,         /// <summary>No button event has occurred.</summary>
    ButtonEvent_Pressed,          /// <summary>The event when button is pressed.</summary>
    ButtonEvent_Released,         /// <summary>The event when button is released.</summary>
    ButtonEvent_Held,             /// <summary>The event when button is being held.</summary>
    ButtonEvent_ReleasedAfterHeld /// <summary>The event when button is released after being held.</summary>
} ButtonEvent;

/// <summary>
///     Data structure for the button state.
/// </summary>
typedef struct {
    int fd;                       /// <summary>File descriptor for the button.</summary>
    bool isPressed;               /// <summary>Whether the button is currently pressed.</summary>
    bool isHeld;                  /// <summary>Whether the button is currently held.</summary>
    struct timespec pressedTime;  /// <summary>The time stamp when the button-press happened.</summary>
} ButtonState;

// Button-related variables
static const time_t buttonHeldThresholdTimeInSeconds = 3L;
static ButtonState button_A_State = {.fd = -1, .isPressed = false, .isHeld = false};
static ButtonState button_B_State = {.fd = -1, .isPressed = false, .isHeld = false};

// LED-related variables
static LedColor LedColor = BLACK;
static LedFlash LedTimer = LED_OFF;
static int Interval;

/// <summary>
///     Set the blink rate of the RGB LED
/// </summary>
/// <param name="LedFlash">Enumerated flash period.</param>
void set_ledblink(LedFlash t)
{
    Interval = 0;
    LedTimer = t;
}

/// <summary>
///     Set the RGB LED color
/// </summary>
/// <param name="LedColor">Enumerated LED color.</param>
void set_ledcolor(LedColor c) 
{
    LedColor = c;
}

/// <summary>
///     Handle LED timer event and manage LED 
/// </summary>
/// <param name="eventData">Context data for handled event.</param>
static void LedTimerEventHandler(EventData *eventData)
{
    switch (LedTimer) {
        case LED_OFF:
            GPIO_SetValue(RedLedGpioFd,GPIO_Value_Low);
            GPIO_SetValue(BlueLedGpioFd,GPIO_Value_Low);
            GPIO_SetValue(GreenLedGpioFd,GPIO_Value_Low);
	    break;

	case LED_BSLOW:
            if( LedColor & 4 ) GPIO_SetValue(RedLedGpioFd, (Interval&2)? GPIO_Value_High:GPIO_Value_Low);
            if( LedColor & 2 ) GPIO_SetValue(GreenLedGpioFd, (Interval&2)? GPIO_Value_High:GPIO_Value_Low);
            if( LedColor & 1 ) GPIO_SetValue(BlueLedGpioFd, (Interval&2)? GPIO_Value_High:GPIO_Value_Low);
	    break;

	case LED_BFAST:
            if( LedColor & 4 ) GPIO_SetValue(RedLedGpioFd,(Interval&1)? GPIO_Value_High:GPIO_Value_Low);
            if( LedColor & 2 ) GPIO_SetValue(GreenLedGpioFd,(Interval&1)? GPIO_Value_High:GPIO_Value_Low);
            if( LedColor & 1 ) GPIO_SetValue(BlueLedGpioFd,(Interval&1)? GPIO_Value_High:GPIO_Value_Low);
	    break;

	case LED_ON:
            if( LedColor & 4 ) GPIO_SetValue(RedLedGpioFd,GPIO_Value_High);
            if( LedColor & 2 ) GPIO_SetValue(GreenLedGpioFd,GPIO_Value_High);
            if( LedColor & 1 ) GPIO_SetValue(BlueLedGpioFd,GPIO_Value_High);
	    break;
        }
    Interval++;
    if( Interval > 3 ) 
        Intervale = 0;
}


ButtonEvent GetButtonEvent(ButtonState *state)
{
    ButtonEvent event = ButtonEvent_None;
    GPIO_Value_Type newInputValue;
    int result = GPIO_GetValue(state->fd, &newInputValue);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        return ButtonEvent_Error;
    }
    if (state->isPressed) {
        if (newInputValue == GPIO_Value_High) {
            // Button has just been released, so set event based on whether button has been held and
            // then reset flags.
            event = state->isHeld ? ButtonEvent_ReleasedAfterHeld : ButtonEvent_Released;
            state->isHeld = false;
            state->isPressed = false;
        }
        if (newInputValue == GPIO_Value_Low && !state->isHeld) {
            // Button has been pressed and hasn't been released yet. As it hasn't been classified as
            // held, compare the elapsed time to determine whether the button has been held long
            // enough to be regarded as 'Held'.
            struct timespec currentTime;
            clock_gettime(CLOCK_REALTIME, &currentTime);
            long elapsedSeconds = currentTime.tv_sec - state->pressedTime.tv_sec;
            state->isHeld = (elapsedSeconds > buttonHeldThresholdTimeInSeconds ||
                             (elapsedSeconds == buttonHeldThresholdTimeInSeconds &&
                              currentTime.tv_nsec >= state->pressedTime.tv_nsec));
            if (state->isHeld) {
                event = ButtonEvent_Held;
            }
        }
    } else if (newInputValue == GPIO_Value_Low) {
        // Button has just been pressed, set isPressed flag and mark current time.
        state->isPressed = true;
        clock_gettime(CLOCK_REALTIME, &state->pressedTime);
        event = ButtonEvent_Pressed;
    }
    return event;
}

// Termination state
static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

static void UpdateBleLedStatus(BleControlMessageProtocolState state)
{
    switch (state) {
        case BleControlMessageProtocolState_Uninitialized:            /// Yellow/constant on - Uninitialized;
        set_ledcolor(YELLOW); 
        set_ledblink(LED_ON);
        break;

        case BleControlMessageProtocolState_AdvertiseToBondedDevices: /// Blue/slow-blink - Advertising to bonded devices only;
        set_ledcolor(BLUE); 
        set_ledblink(LED_BSLOW);
        break;

        case BleControlMessageProtocolState_AdvertisingToAllDevices:  /// Red/slow-blink - Advertising to all devices;
        set_ledcolor(RED); 
        set_ledblink(LED_BSLOW);
        break;

        case BleControlMessageProtocolState_DeviceConnected:          /// Green/slow-blink - Connected to a BLE device;
        set_ledcolor(GREEN); 
        set_ledblink(LED_BSLOW);
        break;

        case BleControlMessageProtocolState_Error:                    /// Magenta/fast-blink - Error
        set_ledcolor(MAGENTA); 
        set_ledblink(LED_BFAST);
        break;

        case BleControlMessageProtocolState_UserLedOn:                /// White/constant on - User LED on (locally initiated) 
        set_ledcolor(WHITE);                                          /// White/slow-blink  - User LED on (remote initiated)
        break;
        }
}

/// <summary>
///     Handle notification of state change generated by the attached BLE device.
/// </summary>
/// <param name="state">The new state of attached BLE device.</param>
static void BleStateChangeHandler(BleControlMessageProtocolState state)
{
    UpdateBleLedStatus(state);
    switch (state) {
    case BleControlMessageProtocolState_Error:
        Log_Debug("INFO: BLE device is in an error state, resetting it...\n");
        set_ledcolor(MAGENTA); 
        set_ledblink(LED_BFAST);
        break;
    case BleControlMessageProtocolState_AdvertiseToBondedDevices:
        Log_Debug("INFO: BLE device is advertising to bonded devices only.\n");
        set_ledcolor(BLUE); 
        set_ledblink(LED_BSLOW);
        break;
    case BleControlMessageProtocolState_AdvertisingToAllDevices:
        Log_Debug("INFO: BLE device is advertising to all devices.\n");
        set_ledcolor(RED); 
        set_ledblink(LED_BSLOW);
        break;
    case BleControlMessageProtocolState_DeviceConnected:
        Log_Debug("INFO: BLE device is now connected to a central device.\n");
        set_ledcolor(GREEN); 
        set_ledblink(LED_BSLOW);
        break;
    case BleControlMessageProtocolState_Uninitialized:
        Log_Debug("INFO: BLE device is now being initialized.\n");
        set_ledcolor(YELLOW); 
        set_ledblink(LED_ON);
        break;
    case BleControlMessageProtocolState_UserLedOn:
        Log_Debug("INFO: BLE device turn on LED.\n");
        set_ledcolor(WHITE); 
        set_ledblink(LED_BSLOW);
        break;
    default:
        Log_Debug("ERROR: Unsupported BLE state: %d.\n", state);
        break;
    }
}

/// <summary>
///     Set the Device Control LED's status.
/// </summary>
/// <param name="state">The LED status to be set.</param>
static void SetDeviceControlLedStatusHandler(bool isOn)
{
    set_ledcolor(WHITE); 
    set_ledblink(isOn? LED_ON:LED_OFF);
}

/// <summary>
///     Get status for the Device Control LED.
/// </summary>
/// <returns>The status of Device Control LED.</returns>
static bool GetDeviceControlLedStatusHandler(void)
{
    return (LedColor == WHITE && LedTimer != LED_OFF);
}

/// <summary>
///     Handle button timer event and take defined actions as printed when the application started.
/// </summary>
/// <param name="eventData">Context data for handled event.</param>
static void ButtonTimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(buttonTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    // Take actions based on button events.
    ButtonEvent button_A_Event = GetButtonEvent(&button_A_State);
    if (button_A_Event == ButtonEvent_Error) {
        terminationRequired = true;
        return;
    } else if (button_A_Event == ButtonEvent_Released) {
        Log_Debug("INFO: SK_BUTTON_A was pressed briefly, allowing new BLE bonds...\n");
        if (BleControlMessageProtocol_AllowNewBleBond(&bleAdvertiseToAllTimeoutPeriod) != 0) {
            Log_Debug("ERROR: Unable to allow new BLE bonds, check nRF52 is connected.\n");
        }
    } else if (button_A_Event == ButtonEvent_Held) {
        Log_Debug("INFO: SK_BUTTON_A is held; deleting all BLE bonds...\n");
        if (BleControlMessageProtocol_DeleteAllBondedDevices() != 0) {
            Log_Debug("ERROR: Unable to delete all BLE bonds, check nRF52 is connected.\n");
        } else {
            Log_Debug("INFO: All BLE bonds are deleted successfully.\n");
        }
    }
    // No actions are defined for other events.

    // Take actions based on SK_BUTTON_B events.
    ButtonEvent button_B_Event = GetButtonEvent(&button_B_State);
    if (button_B_Event == ButtonEvent_Error) {
        terminationRequired = true;
        return;
    } else if (button_B_Event == ButtonEvent_Released) {
        Log_Debug("INFO: SK_BUTTON_B was pressed briefly; toggling SAMPLE_LED.\n");
        SetDeviceControlLedStatusHandler( !GetDeviceControlLedStatusHandler() );
        DeviceControlMessageProtocol_NotifyLedStatusChange();
    } else if (button_B_Event == ButtonEvent_Held) {
        // Forget all stored Wi-Fi networks
        Log_Debug("INFO: SK_BUTTON_B is held; forgetting all stored Wi-Fi networks...\n");
        if (WifiConfig_ForgetAllNetworks() != 0) {
            Log_Debug("ERROR: Unable to forget all stored Wi-Fi networks: %s (%d).\n",
                      strerror(errno), errno);
        } else {
            Log_Debug("INFO: All stored Wi-Fi networks are forgotten successfully.\n");
        }
    }
    // No actions are defined for other events.
}

// event handler data structures. Only the event handler field needs to be populated.
static EventData buttonsEventData = {.eventHandler = &ButtonTimerEventHandler};
static EventData ledEventData = {.eventHandler = &LedTimerEventHandler};

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure.</returns>
static int InitPeripheralsAndHandlers(void)
{
    // Open the GPIO controlling the nRF52 reset pin, and keep it held in reset (low) until needed.
    bleDeviceResetPinGpioFd = GPIO_OpenAsOutput(SAMPLE_NRF52_RESET, GPIO_OutputMode_OpenDrain, GPIO_Value_Low);
    if (bleDeviceResetPinGpioFd < 0) {
        Log_Debug("ERROR: Could not open GPIO 5 as reset pin: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

    // Open the UART and set up UART event handler.
    UART_Config uartConfig;
    UART_InitConfig(&uartConfig);
    uartConfig.baudRate = 115200;
    uartConfig.flowControl = UART_FlowControl_RTSCTS;
    uartFd = UART_Open(SAMPLE_NRF52_UART, &uartConfig);
    if (uartFd < 0) {
        Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
        return -1;
    }
    if (MessageProtocol_Init(epollFd, uartFd) < 0) {
        return -1;
    }

    BleControlMessageProtocol_Init(BleStateChangeHandler, epollFd);
    WifiConfigMessageProtocol_Init();
    DeviceControlMessageProtocol_Init(SetDeviceControlLedStatusHandler,GetDeviceControlLedStatusHandler);

    Log_Debug("Opening SK_BUTTON_A as input\n");
    button_A_State.fd = GPIO_OpenAsInput(SK_BUTTON_A);
    if (button_A_State.fd < 0) {
        Log_Debug("ERROR: Could not open SK_BUTTON_A GPIO: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

    Log_Debug("Opening SK_BUTTON_B as input.\n");
    button_B_State.fd = GPIO_OpenAsInput(SK_BUTTON_B);
    if (button_B_State.fd < 0) {
        Log_Debug("ERROR: Could not open SK_BUTTON_B GPIO: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

    struct timespec buttonStatusCheckPeriod = {0, 1000000};
    buttonTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &buttonStatusCheckPeriod, &buttonsEventData, EPOLLIN);
    if (buttonTimerFd < 0) {
        return -1;
    }

    // Open Red RGB LED GPIO and set as output with value GPIO_Value_High (off).
    Log_Debug("Opening RGB LED RED Led for output\n");
    RedLedGpioFd= GPIO_OpenAsOutput(SAMPLE_RGBLED_BLUE, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (RedLedGpioFd< 0) {
        Log_Debug("ERROR: Could not open SAMPLE_RGBLED_BLUE GPIO: %s (%d).\n", strerror(errno),
                  errno);
        return -1;
    }

    // Open Green RGB LED GPIO and set as output with value GPIO_Value_High (off).
    Log_Debug("Opening RGB GREEN Led for output\n");
    GreenLedGpioFd = GPIO_OpenAsOutput(SAMPLE_RGBLED_RED, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (GreenLedGpioFd< 0) {
        Log_Debug("ERROR: Could not open SAMPLE_RGBLED_RED GPIO: %s (%d).\n", strerror(errno),
                  errno);
        return -1;
    }

    // Open Blue RGB LED GPIO and set as output with value GPIO_Value_High (off).
    Log_Debug("Opening RGB LED BLUE Led for output\n");
    BlueLedGpioFd = GPIO_OpenAsOutput(SAMPLE_RGBLED_GREEN, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (BlueLedGpioFd< 0) {
        Log_Debug("ERROR: Could not open SAMPLE_RGBLED_GREEN GPIO: %s (%d).\n", strerror(errno),
                  errno);
        return -1;
    }

    set_ledblink(LED_OFF);
    set_ledcolor(BLACK);

    struct timespec ledServicePeriod = {0, 500000};
    ledTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &ledServicePeriod, &ledEventData, EPOLLIN);
    if (ledTimerFd < 0) {
        return -1;
    }

    UpdateBleLedStatus(BleControlMessageProtocolState_Uninitialized);

    // Initialization completed, start the nRF52 application.
    GPIO_SetValue(bleDeviceResetPinGpioFd, GPIO_Value_High);

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    // Leave the LED off
    if (RedLedGpioFd >= 0) {
        GPIO_SetValue(RedLedGpioFd, GPIO_Value_High);
    }
    if (GreenLedGpioFd >= 0) {
        GPIO_SetValue(GreenLedGpioFd, GPIO_Value_High);
    }
    if (BlueLedGpioFd >= 0) {
        GPIO_SetValue(BlueLedGpioFd, GPIO_Value_High);
    }

    Log_Debug("Closing file descriptors\n");
    CloseFdAndPrintError(buttonTimerFd, "ButtonTimer");
    CloseFdAndPrintError(button_A_State.fd, "Button1");
    CloseFdAndPrintError(button_B_State.fd, "Button2");
    CloseFdAndPrintError(bleDeviceResetPinGpioFd, "BleDeviceResetPin");
    CloseFdAndPrintError(RedLedGpioFd, "BleAdvertiseToBondedDevicesLed");
    CloseFdAndPrintError(GreenLedGpioFd, "BleAdvertiseToAllDevicesLed");
    CloseFdAndPrintError(BlueLedGpioFd, "BleConnectedLed");
    CloseFdAndPrintError(epollFd, "Epoll");
    CloseFdAndPrintError(uartFd, "Uart");
    DeviceControlMessageProtocol_Cleanup();
    WifiConfigMessageProtocol_Cleanup();
    BleControlMessageProtocol_Cleanup();
    MessageProtocol_Cleanup();
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("INFO: BLE Wi-Fi application starting.\n");
    Log_Debug(
        "Available actions on the Azure Sphere device:\n"
        "\tPress SK_BUTTON_A  - Start allowing new BLE bonds for 1 minute\n"
        "\tHold SK_BUTTON_A   - Delete all BLE bonds\n"
        "\tPress SK_BUTTON_B  - Toggle SAMPLE_LED\n"
        "\tHold SK_BUTTON_B   - Forget all stored Wi-Fi networks on Azure Sphere device\n\n"
        "SAMPLE_RGBLED's color indicates BLE status for the attached nRF52 board:\n"
        "\tYellow  - Uninitialized\n"
        "\tBlue    - Advertising to bonded devices only\n"
        "\tRed     - Advertising to all devices\n"
        "\tGreen   - Connected to a central device\n"
        "\tMagenta - Error\n\n");
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("INFO: Application exiting\n");
    return 0;
}
