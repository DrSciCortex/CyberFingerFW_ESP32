#pragma once
#include <stdint.h>

class XboxGamepadDevice; 

// D-pad / Hat convention (common):
// 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW, 8=CENTER
static constexpr uint8_t DPAD_CENTER = 8;

// Canonical half packet sent by BOTH halves.
// Each half fills what it physically has. The receiver remaps by origin.

typedef struct __attribute__((packed))
{
    uint8_t ver;      // CYBER_PROTO_VER
    uint8_t srcRole;  // 0=left, 1=right
    uint16_t seq;     // rolling sequence number
    uint32_t ms;      // millis() at sender
    uint16_t btnMask; // canonical digital bits (AX,BY,ST,BP,STARTSELECT,... optional)
    int16_t jx;       // originating stick X
    int16_t jy;       // originating stick Y
    uint8_t trigger; // analog trigger 0..255 (if this half has LT; else 0)
    uint8_t dpad; // 0..8 hat; if this half has dpad; else DPAD_CENTER
} HalfPacket;



// Canonical packet bits
static constexpr uint16_t PKT_AX = (1u << 0);          // left=X, right=A
static constexpr uint16_t PKT_BY = (1u << 1);          // left=Y, right=B
static constexpr uint16_t PKT_ST = (1u << 2);           // left=LS, right=RS
static constexpr uint16_t PKT_BP = (1u << 3);           // left=LB, right=RB
static constexpr uint16_t PKT_STARTSELECT = (1u << 4); // left=SELECT, right=START

// Merged HID-facing digital buttons (bitmask internal to this class)
enum HidBtn : uint16_t
{
    H_A = (1u << 0),
    H_B = (1u << 1),
    H_X = (1u << 2),
    H_Y = (1u << 3),
    H_LB = (1u << 4),
    H_RB = (1u << 5),
    H_LS = (1u << 6),
    H_RS = (1u << 7),
    H_SELECT = (1u << 8),
    H_START = (1u << 9),
};

class GamepadMerged
{
public:
    struct Config
    {
        int stickDeadzone = 250; // region considered zero for stick
        int triggerDeadzone = 2; // region considered zero for trigger
        int stickDeltaThreshold = 150; // changes less than this are not transmitted.
        int triggerDeltaThreshold = 2;

        bool debug = false;
    };

    GamepadMerged(XboxGamepadDevice *gamepadPtr, const Config& cfg);
    GamepadMerged(XboxGamepadDevice* gamepadPtr);

    // RIGHT side only
    void setRightLocal(const HalfPacket &right);
    void setLeftRemote(const HalfPacket &left);

    // does merge + diff + emits events + sendGamepadReport() once if needed
    void updateAndSendIfChanged();

    void setGamepad(XboxGamepadDevice* gamepadPtr);
    void setConfig(const Config& cfg);

    // LEFT side only
    void setLeftLocal(const HalfPacket &left);

    // Build a packet for ESPNOW if left-local state changed (deltaThreshold aware).
    // Returns true if outPkt is valid and should be sent.
    bool buildLeftPacketIfChanged(HalfPacket &outPkt);

    // Convenience: build + send (returns true if actually sent)
    bool sendLeftPacketToRightIfChanged(const uint8_t peerMac[6]);
    bool sendLeftPacketToRight(const uint8_t peerMac[6]);

private:
    struct MergedState
    {
        uint16_t buttons = 0; // HidBtn mask
        int16_t lx = 0, ly = 0;
        int16_t rx = 0, ry = 0;
        uint8_t lt = 0, rt = 0;
        uint8_t dpad = DPAD_CENTER;
    };

    static inline void setBit(uint16_t &dst, uint16_t bit, bool on)
    {
        if (on)
            dst |= bit;
        else
            dst &= ~bit;
    }

    void merge_();

    bool emitButtonEvents_();
    bool emitStickEvents_();
    bool emitTriggerEvents_();
    // bool emitDpadEvents_();

    void press_(uint16_t hidBit);
    void release_(uint16_t hidBit);

    // single point of adaptation for your library’s exact API names
    void setLeftThumb_(int16_t x, int16_t y);
    void setRightThumb_(int16_t x, int16_t y);
    void setTriggers_(uint8_t lt, uint8_t rt);
    // void setDpad_(uint8_t hat);
    void sendReport_();

    XboxGamepadDevice *gamepad_;
    Config cfg_;

    HalfPacket left_{};
    HalfPacket right_{};
    bool leftValid_ = false;
    bool rightValid_ = false;

    MergedState cur_{};
    MergedState prev_{};

    // left-side send state
    HalfPacket leftSentPrev_{};     // last packet we actually sent (for diff)
    bool leftHasPrevSent_ = false;
    uint32_t leftLastSendMs_ = 0;   // optional throttle if you want later
    uint16_t leftSeq_ = 0;       

};
