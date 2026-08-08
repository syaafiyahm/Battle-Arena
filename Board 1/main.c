/*=============================================================================
    Battle Arena : Multiplayer Fighting Game over UART
    Target      : Nuvoton NUC140VE3AN (NuTiny / LB-002 Learning Board)
    Toolchain   : Keil uVision (ARM-ADS, armcc)

    Flash this SAME .c file to BOTH boards - it works for either player.
    Connect the two boards' UART0 TX/RX/GND together (cross TX<->RX).

    -------------------------------------------------------------------------
    !! THINGS YOU MUST VERIFY / EDIT FOR YOUR OWN BOARD (search "TODO") !!
    -------------------------------------------------------------------------
    1. LED_PORT / LED_xxx_PIN and BUZZER_PORT / BUZZER_PIN - I don't have your
       schematic, so these are placeholders. Change them to match your LB-002
       wiring.
    2. KEY_NONE - the "no key pressed" return value of ScanKey(). I don't have
       ScanKey.c, so this is my best-guess default. If keys don't register or
       register when nothing is pressed, open your working keypad demo
       (haha2) and check what ScanKey() returns when idle, then fix KEY_NONE
       and the KEY_1..KEY_4 defines below to match your matrix layout.
    3. UART wiring - this assumes UART_PORT0 pins are muxed via
       DrvGPIO_InitFunction(E_FUNC_UART0_RX_TX). If your board uses UART1/2,
       change UART_CH and the InitFunction call.
=============================================================================*/

#include <stdio.h>
#include "NUC1xx.h"
#include "SYS.h"
#include "GPIO.h"
#include "UART.h"
#include "LCD.h"
#include "Scankey.h"
#include "Seven_Segment.h"

/*-----------------------------------------------------------------------
  Board / pin configuration  (TODO: confirm against your LB-002 wiring)
-----------------------------------------------------------------------*/
#define UART_CH             UART_PORT0
#define UART_BAUD           115200

#define LED_PORT            E_GPB
#define LED_GREEN_PIN       0          /* HP > 70                    */
#define LED_YELLOW_PIN      1          /* 30 <= HP <= 70              */
#define LED_RED_PIN         2          /* HP < 30 (blinks)            */

#define BUZZER_PORT         E_GPB
#define BUZZER_PIN          3

/*-----------------------------------------------------------------------
  Keypad mapping (TODO: confirm against ScanKey.c / haha2 demo)
-----------------------------------------------------------------------*/
#define KEY_NONE            0xFF
#define KEY_1               1          /* Punch          -5 HP        */
#define KEY_2               2          /* Kick           -10 HP       */
#define KEY_3               3          /* Special Attack -20 HP       */
#define KEY_4               4          /* Block          -50% dmg     */

/*-----------------------------------------------------------------------
  Game constants
-----------------------------------------------------------------------*/
#define START_HP            100
#define MATCH_SECONDS       60

#define DMG_PUNCH           5
#define DMG_KICK            10
#define DMG_SPECIAL         20

#define SPECIAL_COOLDOWN_S  10
#define BLOCK_COOLDOWN_S    5
#define BLOCK_ACTIVE_S      2

/*-----------------------------------------------------------------------
  UART packet protocol   [ CMD , DATA ]   (2 bytes per packet)
-----------------------------------------------------------------------*/
#define CMD_START           0x01
#define CMD_PUNCH           0x02
#define CMD_KICK            0x03
#define CMD_SPECIAL         0x04
#define CMD_BLOCK           0x05
#define CMD_HP_UPDATE       0x06
#define CMD_GAME_OVER       0x07
#define CMD_RESTART         0x08

/*-----------------------------------------------------------------------
  Game state machine
-----------------------------------------------------------------------*/
typedef enum
{
    STATE_INTRO,
    STATE_WAIT_OPPONENT,
    STATE_COUNTDOWN,
    STATE_BATTLE,
    STATE_GAMEOVER
} E_GAME_STATE;

static E_GAME_STATE g_state;

static int16_t  g_myHP,    g_enemyHP;
static uint16_t g_timeLeft;              /* seconds remaining          */
static uint8_t  g_specialCooldown;       /* seconds left before usable */
static uint8_t  g_blockCooldown;
static uint8_t  g_blockActiveTicks;      /* seconds the shield still holds */
static uint8_t  g_lastKey = KEY_NONE;    /* for simple debounce (edge detect) */

static char     g_lcdLine[17];
static char     g_statusMsg[17] = "";

/*=============================================================================
    Low level helpers
=============================================================================*/

/* Blocking, but broken into small chunks so keypad/UART are polled during
   any "wait roughly N ms" pause instead of freezing the whole board.
   TODO: replace with a hardware Timer interrupt (Timer.c/h) for a
   production-quality non-blocking 1 Hz tick if you want it fully async. */
static void Delay_ms(uint32_t ms)
{
    DrvSYS_Delay(ms * 1000UL);
}

static void UART_SendPacket(uint8_t cmd, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = cmd;
    buf[1] = data;
    DrvUART_Write(UART_CH, buf, 2);
}

/* Non-blocking single-byte read (used for the un-framed handshake, where
   we just need to spot a single marker byte anywhere in the stream).
   Uses the raw register macro directly instead of DrvUART_Read(), since
   DrvUART_Read() was observed reporting failure even when
   _DRVUART_RECEIVEAVAILABLE() shows bytes waiting in the FIFO. */
static int32_t UART_TryReceiveByte(uint8_t *outByte)
{
    if (_DRVUART_RECEIVEAVAILABLE(UART_CH) < 1)
        return 0;

    *outByte = (uint8_t)_DRVUART_RECEIVEBYTE(UART_CH);
    return 1;
}

/* Non-blocking receive: returns 1 and fills cmd/data if a full 2 byte
   packet is available, otherwise returns 0 immediately. Built on top of
   UART_TryReceiveByte() (the raw register read) rather than DrvUART_Read()
   for the same reliability reason. */
static int32_t UART_TryReceivePacket(uint8_t *cmd, uint8_t *data)
{
    if (_DRVUART_RECEIVEAVAILABLE(UART_CH) < 2)
        return 0;

    if (!UART_TryReceiveByte(cmd))
        return 0;

    if (!UART_TryReceiveByte(data))
        return 0;

    return 1;
}

static void Buzzer_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t repeat)
{
    uint8_t i;
    for (i = 0; i < repeat; i++)
    {
        DrvGPIO_SetBit(BUZZER_PORT, BUZZER_PIN);
        Delay_ms(on_ms);
        DrvGPIO_ClrBit(BUZZER_PORT, BUZZER_PIN);
        if (off_ms) Delay_ms(off_ms);
    }
}

static void LED_UpdateForHP(int16_t hp)
{
    if (hp <= 0)
    {
        /* all LEDs flash - handled by caller loop, just force all on here */
        DrvGPIO_SetBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_SetBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_SetBit(LED_PORT, LED_RED_PIN);
        return;
    }

    if (hp > 70)
    {
        DrvGPIO_SetBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
    }
    else if (hp >= 30)
    {
        DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_SetBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
    }
    else
    {
        DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
        /* simple blink using system tick parity */
        if ((g_timeLeft & 1) == 0)
            DrvGPIO_SetBit(LED_PORT, LED_RED_PIN);
        else
            DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
    }
}

/* Matches the working pattern from Smpl_7seg's DisplayCounter(): each
   digit is only lit while its enable pin is held, so we must close then
   re-show each digit in turn. This function must be called REPEATEDLY
   (many times a second) for the display to stay visibly lit via
   persistence of vision - a single call per second is not enough. */
static void ShowTimer(uint16_t seconds)
{
    uint8_t tens = (uint8_t)(seconds / 10);
    uint8_t ones = (uint8_t)(seconds % 10);

    close_seven_segment();
    show_seven_segment(1, tens);
    DrvSYS_Delay(1200);

    close_seven_segment();
    show_seven_segment(0, ones);
    DrvSYS_Delay(1200);
}

static void RefreshLCD(char *myStatus)
{
    clear_LCD();

    sprintf(g_lcdLine, "P:%3d E:%3d", g_myHP, g_enemyHP);
    print_Line(0, g_lcdLine);

    sprintf(g_lcdLine, "Time: %2d", g_timeLeft);
    print_Line(1, g_lcdLine);

    print_Line(3, myStatus);
}

/*=============================================================================
    System bring-up
=============================================================================*/
static void Hardware_Init(void)
{
    STR_UART_T uartCfg;

    /* Clock/system init already happens automatically via SystemInit()
       (in system_NUC1xx.c), which the CMSIS startup code calls before
       main() runs - no manual SYS_Init() call needed/available here. */

    /* --- UART0 pin mux + open --- */
    DrvGPIO_InitFunction(E_FUNC_UART0_RX_TX);   /* TODO: change if using UART1/2 */

    uartCfg.u32BaudRate       = UART_BAUD;
    uartCfg.u8cDataBits       = DRVUART_DATABITS_8;
    uartCfg.u8cStopBits       = DRVUART_STOPBITS_1;
    uartCfg.u8cParity         = DRVUART_PARITY_NONE;
    uartCfg.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
    uartCfg.u8TimeOut         = 0;
    DrvUART_Open(UART_CH, &uartCfg);

    /* --- LEDs --- */
    DrvGPIO_Open(LED_PORT, LED_GREEN_PIN,  E_IO_OUTPUT);
    DrvGPIO_Open(LED_PORT, LED_YELLOW_PIN, E_IO_OUTPUT);
    DrvGPIO_Open(LED_PORT, LED_RED_PIN,    E_IO_OUTPUT);
    DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
    DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
    DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);

    /* --- Buzzer --- */
    DrvGPIO_Open(BUZZER_PORT, BUZZER_PIN, E_IO_OUTPUT);
    DrvGPIO_ClrBit(BUZZER_PORT, BUZZER_PIN);

    /* --- 7-Segment (matches Init_RotateSeg() from Smpl_7seg, plus
           GPC4/5 explicitly since we also drive digit indices 0 and 1
           which that demo left implicit) --- */
    DrvGPIO_Open(E_GPC, 4, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPC, 5, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPC, 6, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPC, 7, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 0, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 2, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 3, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 4, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 5, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 6, E_IO_OUTPUT);
    DrvGPIO_Open(E_GPE, 7, E_IO_OUTPUT);
    close_seven_segment();   /* blank all digits at boot */

    /* --- LCD + Keypad --- */
    init_LCD();
    clear_LCD();
    OpenKeyPad();
}

/*=============================================================================
    Keypad helper - returns a NEWLY pressed key once (edge-triggered),
    or KEY_NONE if nothing new was pressed since the last call.
=============================================================================*/
static uint8_t GetNewKeyPress(void)
{
    uint8_t key = ScanKey();
    uint8_t pressed = KEY_NONE;

    if (key != KEY_NONE && key != g_lastKey)
        pressed = key;

    g_lastKey = key;
    return pressed;
}

/*=============================================================================
    Game state handlers
=============================================================================*/
static void ResetMatch(void)
{
    g_myHP            = START_HP;
    g_enemyHP         = START_HP;
    g_timeLeft        = MATCH_SECONDS;
    g_specialCooldown = 0;
    g_blockCooldown   = 0;
    g_blockActiveTicks= 0;
    g_lastKey         = KEY_NONE;
}

static void DoIntro(void)
{
    print_Line(0, "== BATTLE ARENA ==");
    print_Line(2, "Press any key");
    print_Line(3, "to start match");

    if (GetNewKeyPress() != KEY_NONE)
    {
        ResetMatch();
        UART_SendPacket(CMD_START, 0);
        g_state = STATE_WAIT_OPPONENT;
        clear_LCD();
    }
}

static void DoWaitOpponent(void)
{
    uint8_t rxByte;
    uint32_t rxAvail;
    uint8_t got = 0;

    /* NOTE: this LCD only shows ~4 text lines (rows 0-3) at once, so the
       debug info below uses only those rows instead of 5/6 which sit
       off the bottom edge of the 64px-tall screen. */

    print_Line(0, "BATTLE ARENA");
    print_Line(1, "Waiting opp...");

    /* --- DEBUG: show raw bytes sitting in the UART RX FIFO -------------
       Remove this block once communication is confirmed working. -------*/
    rxAvail = _DRVUART_RECEIVEAVAILABLE(UART_CH);
    sprintf(g_lcdLine, "RX:%lu", (unsigned long)rxAvail);
    print_Line(2, g_lcdLine);
    /* ------------------------------------------------------------------*/

    /* keep re-announcing START in case the other board booted later */
    UART_SendPacket(CMD_START, 0);

    /* Handshake only needs to spot the CMD_START marker byte ANYWHERE in
       the incoming stream - scanning byte-by-byte instead of reading
       fixed 2-byte pairs means we can't get stuck out of alignment. */
    while (UART_TryReceiveByte(&rxByte))
    {
        /* --- DEBUG: show the last raw byte actually received --------- */
        sprintf(g_lcdLine, "Last:0x%02X", rxByte);
        print_Line(3, g_lcdLine);
        /* --------------------------------------------------------------*/

        if (rxByte == CMD_START)
        {
            got = 1;
            break;
        }
    }

    if (got)
    {
        g_state = STATE_COUNTDOWN;
        clear_LCD();
        return;
    }

    Delay_ms(200);   /* avoid flooding the UART/LCD while waiting */
}

static void DoCountdown(void)
{
    int8_t i;
    close_seven_segment();   /* blank leftover digit from previous match */
    for (i = 3; i >= 1; i--)
    {
        clear_LCD();
        sprintf(g_lcdLine, "%d...", i);
        print_Line(3, g_lcdLine);
        Buzzer_Beep(100, 0, 1);
        Delay_ms(700);
    }
    clear_LCD();
    print_Line(3, "FIGHT!!");
    Buzzer_Beep(300, 0, 1);
    Delay_ms(500);

    clear_LCD();
    g_state = STATE_BATTLE;
}

static void ApplyIncomingDamage(uint8_t dmg)
{
    if (g_blockActiveTicks > 0)
        dmg = (uint8_t)(dmg / 2);          /* block halves damage */

    g_myHP -= dmg;
    if (g_myHP < 0) g_myHP = 0;

    Buzzer_Beep(60, 0, 1);                 /* "hit" beep */
    UART_SendPacket(CMD_HP_UPDATE, (uint8_t)g_myHP);
}

static void DoBattle_HandleKey(uint8_t key)
{
    switch (key)
    {
        case KEY_1: /* Punch */
            UART_SendPacket(CMD_PUNCH, DMG_PUNCH);
            sprintf(g_statusMsg, "You: PUNCH!");
            break;

        case KEY_2: /* Kick */
            UART_SendPacket(CMD_KICK, DMG_KICK);
            sprintf(g_statusMsg, "You: KICK!");
            break;

        case KEY_3: /* Special Attack (cooldown) */
            if (g_specialCooldown == 0)
            {
                UART_SendPacket(CMD_SPECIAL, DMG_SPECIAL);
                Buzzer_Beep(400, 0, 1);
                g_specialCooldown = SPECIAL_COOLDOWN_S;
                sprintf(g_statusMsg, "You: SPECIAL!");
            }
            else
            {
                sprintf(g_statusMsg, "Special on CD");
            }
            break;

        case KEY_4: /* Block (cooldown) */
            if (g_blockCooldown == 0)
            {
                g_blockActiveTicks = BLOCK_ACTIVE_S;
                g_blockCooldown    = BLOCK_COOLDOWN_S;
                UART_SendPacket(CMD_BLOCK, 0);
                sprintf(g_statusMsg, "You: BLOCK!");
            }
            else
            {
                sprintf(g_statusMsg, "Block on CD");
            }
            break;

        default:
            break;
    }
}

static void DoBattle_HandlePacket(uint8_t cmd, uint8_t data)
{
    switch (cmd)
    {
        case CMD_PUNCH:
        case CMD_KICK:
        case CMD_SPECIAL:
            ApplyIncomingDamage(data);
            sprintf(g_statusMsg, "OUCH!!");
            break;

        case CMD_BLOCK:
            /* purely informational - opponent is shielding, nothing to do */
            break;

        case CMD_HP_UPDATE:
            g_enemyHP = data;
            break;

        case CMD_GAME_OVER:
            /* opponent ended the match (their HP hit 0, or their timer) */
            g_state = STATE_GAMEOVER;
            break;

        default:
            break;
    }
}

static void DoBattle(void)
{
    uint8_t key, cmd, data;
    uint16_t chunk;

    g_statusMsg[0] = '\0';   /* clear last status at the start of each second */

    /* --- one "second" of battle, sliced into short chunks so we keep
           polling keypad + UART instead of freezing for a whole second --- */
    for (chunk = 0; chunk < 10; chunk++)
    {
        key = GetNewKeyPress();
        if (key != KEY_NONE)
            DoBattle_HandleKey(key);

        if (UART_TryReceivePacket(&cmd, &data))
            DoBattle_HandlePacket(cmd, data);

        RefreshLCD(g_statusMsg);
        LED_UpdateForHP(g_myHP);

        if (g_myHP <= 0 || g_enemyHP <= 0)
            break;

        ShowTimer(g_timeLeft);   /* refresh every chunk (~10x/sec) so it stays lit */
        Delay_ms(100);   /* 10 x 100ms = ~1s per outer tick */
    }

    /* --- 1 Hz bookkeeping --- */
    if (g_timeLeft > 0) g_timeLeft--;
    if (g_specialCooldown > 0)  g_specialCooldown--;
    if (g_blockCooldown > 0)    g_blockCooldown--;
    if (g_blockActiveTicks > 0) g_blockActiveTicks--;

    if (g_myHP <= 0 || g_enemyHP <= 0 || g_timeLeft == 0)
    {
        UART_SendPacket(CMD_GAME_OVER, 0);
        g_state = STATE_GAMEOVER;
    }
}

static void DoGameOver(void)
{
    close_seven_segment();   /* leave the display clean, no half-lit digit */
    clear_LCD();
    print_Line(0, "GAME OVER");

    if (g_myHP <= 0 && g_enemyHP > 0)
    {
        print_Line(1, "YOU LOSE");
        Buzzer_Beep(250, 150, 3);
    }
    else if (g_enemyHP <= 0 && g_myHP > 0)
    {
        print_Line(1, "YOU WIN!");
        Buzzer_Beep(120, 100, 3);
    }
    else if (g_myHP > g_enemyHP)
    {
        print_Line(1, "YOU WIN!");
        Buzzer_Beep(120, 100, 3);
    }
    else if (g_myHP < g_enemyHP)
    {
        print_Line(1, "YOU LOSE");
        Buzzer_Beep(250, 150, 3);
    }
    else
    {
        print_Line(1, "DRAW");
        Buzzer_Beep(200, 200, 2);
    }

    print_Line(3, "Key = restart");

    DrvGPIO_SetBit(LED_PORT, LED_GREEN_PIN);
    DrvGPIO_SetBit(LED_PORT, LED_YELLOW_PIN);
    DrvGPIO_SetBit(LED_PORT, LED_RED_PIN);

    if (GetNewKeyPress() != KEY_NONE)
    {
        UART_SendPacket(CMD_RESTART, 0);
        DrvGPIO_ClrBit(LED_PORT, LED_GREEN_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_YELLOW_PIN);
        DrvGPIO_ClrBit(LED_PORT, LED_RED_PIN);
        g_state = STATE_INTRO;
        clear_LCD();
    }
}

/*=============================================================================
    main
=============================================================================*/
int main(void)
{
    Hardware_Init();

    g_state = STATE_INTRO;

    while (1)
    {
        switch (g_state)
        {
            case STATE_INTRO:          DoIntro();          break;
            case STATE_WAIT_OPPONENT:  DoWaitOpponent();   break;
            case STATE_COUNTDOWN:      DoCountdown();      break;
            case STATE_BATTLE:         DoBattle();         break;
            case STATE_GAMEOVER:       DoGameOver();       break;
        }
    }
}
