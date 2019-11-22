#pragma ones

#include <stdint.h>

namespace umouse {

    class LED {

      public:
        LED();
        virtual ~LED();
        void update();
        virtual void setState(bool state_);
        bool getState();
        void turn(bool state_);
        void flash(uint16_t on_count, uint16_t off_count);

      protected:
        volatile bool state;
        volatile uint16_t LED_count; //LEDの点滅用カウント
        volatile uint16_t on_count_LED; //LEDの点滅時間のon時間のカウント
        volatile uint16_t off_count_LED; //LEDの点滅時間のoff時間のカウント
        volatile uint8_t flag_flash_LED; //LEDが点滅状態か常時点灯状態かのフラグ 1:点滅  0:常時点灯(消灯も含む)
    };

}
