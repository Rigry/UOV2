#define STM32F030x6
#define F_OSC   8000000UL
#define F_CPU   48000000UL

#include "init_clock.h"
#include "periph_rcc.h"
#include "periph_flash.h"
#include "flash.h"
#include "pin.h"
#include "adc.h"
#include "button.h"
#include "hysteresis.h"
#include "NTC_table.h"
#include "timers.h"
// #include "sensor.h"

/// эта функция вызывается первой в startup файле
extern "C" void init_clock () { init_clock<F_OSC,F_CPU>(); }

using EPRA     = mcu::PA3;
using UV_ON    = mcu::PA1;
using EN_UV    = mcu::PB0;
using UV_WORK  = mcu::PA12; 
using UV_ALARM = mcu::PA11;
using LEVEL    = mcu::PA10;

using UZ       = mcu::PA6;
using UZ_ON    = mcu::PA0;
using EN_UZ    = mcu::PB1;
using UZ_WORK  = mcu::PA9;
using UZ_ALARM = mcu::PA8;

using OVERHEAT = mcu::PB15;

using RC       = mcu::PB2;
using RC_ON    = mcu::PB10;

int main()
{
   struct Flash_data {
      uint16_t max_uv_level = 0;
   }flash;

   [[maybe_unused]] auto _ = Flash_updater<
        mcu::FLASH::Sector::_11
      , mcu::FLASH::Sector::_10
   >::make (&flash);

   auto[epra, uz, en_uv, en_uz, rc, on] = make_pins<mcu::PinMode::Input, EPRA, UZ, EN_UV, EN_UZ, RC, RC_ON>();
   auto[uv_on, uv_work, uv_alarm, level, uz_on, uz_work, uz_alarm, overheat] 
        = make_pins<mcu::PinMode::Output, UV_ON, UV_WORK, UV_ALARM, LEVEL, UZ_ON, UZ_WORK, UZ_ALARM, OVERHEAT>();

   // bool on{false};
   // auto rc_on = Button<RC_ON>();
   // rc_on.set_down_callback([&]{ on ^= 1;});

   constexpr auto conversion_on_channel {16};
   struct ADC_{
      ADC_average& control     = ADC_average::make<mcu::Periph::ADC1>(conversion_on_channel);
      ADC_channel& temperature = control.add_channel<mcu::PA4>();
      ADC_channel& uv_level    = control.add_channel<mcu::PA5>();
   }adc;

   adc.control.start();

   const size_t U = 33;
   const size_t R = 5100;
   
   volatile uint16_t temperature{0};
   auto temp = [&](uint16_t adc) {
      adc = adc / conversion_on_channel;
      auto p = std::lower_bound(
         std::begin(NTC::u2904<U,R>),
         std::end(NTC::u2904<U,R>),
         adc,
         std::greater<uint32_t>());
      temperature = (p - NTC::u2904<U,R>);
   };

   Timer delay_level{};
   Timer delay_epra{};
   bool level_delay{false};
   bool epra_delay{false};

   while(1){

      if ((on or en_uv) and not level_delay and not epra_delay) {
         delay_level.start(120000);
         delay_epra.start(30000);
      }

      if (delay_level.done() and not level_delay) {
         level_delay = true;
         delay_level.stop();
      }

      if (delay_epra.done() and not epra_delay) {
         epra_delay = true;
         delay_epra.stop();
      }

      temp(adc.temperature);
      overheat = Hysteresis(temperature, 20, 40);

      flash.max_uv_level = adc.uv_level > flash.max_uv_level ? adc.uv_level : flash.max_uv_level;
      
      if (not rc) {
         on = false;
         
         uv_work = uv_on = (en_uv and not overheat);
         uz_work = uz_on = (en_uz and not overheat);

         if (adc.uv_level > (flash.max_uv_level * 0.45)) {
            level = false;
         } else if (adc.uv_level < (flash.max_uv_level * 0.4)){
            level = (en_uv & (level_delay));
            level = level & true;
         }
         
         uv_alarm = (en_uv & not epra and epra_delay);
         uz_alarm = (en_uz & not uz);
      } else {
         en_uv = false;
         uv_work = uv_on = (on and not overheat);
         uz_work = uz_on = (on and not overheat);

         // level = (on & (level_delay and (adc.uv_level < (flash.max_uv_level * 0.4)) ));
         if (adc.uv_level > (flash.max_uv_level * 0.45)) {
            level = false;
         } else if (adc.uv_level < (flash.max_uv_level * 0.4)){
            level = (on & (level_delay));
            level = level & true;
         }
         uv_alarm = (on & not epra and epra_delay);
         uz_alarm = (on & not uz);
      }

      if (rc and not on) {
         level_delay = false;
         delay_level.stop();
         epra_delay = false;
         delay_epra.stop();
      }

      if (not en_uv and not rc) {
         level_delay = false;
         delay_level.stop();
         epra_delay = false;
         delay_epra.stop();
      }

      __WFI();
   }

}



