// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// User interface.

#include "rings/ui.h"

#include <algorithm>
#include <stdint.h>  //MATO TU BÝT ???? 
#include "stmlib/system/system_clock.h"
//#include "stmlib/system/storage.h" // Nezávislá perzistentní paměť FLASH tohle bylo přidáno
#include "rings/cv_scaler.h"
#include "rings/dsp/part.h"
#include "rings/dsp/string_synth_part.h"

namespace rings {

const int32_t kLongPressDuration = 3000;

using namespace std;
using namespace stmlib;

// Definice struktury pro perzistentní úložiště serva (aby kompilátor věděl, co ukládá)
//struct ServoStorageData {
//  uint16_t dac_offset_backup;
 // uint16_t reserved;
//};

// Inicializace nezávislého úložiště mimo tónové settings
//stmlib::Storage<2> servo_storage;
//static ServoStorageData servo_flash_data;
//static uint16_t servo_version_token = 0;

// Globální stavové proměnné pro perzistentní auto-ukládání serva
//bool servo_data_changed = false; 
//static int32_t save_timeout __attribute__((unused)) = 0;

void Ui::Init(
    Settings* settings,
    CvScaler* cv_scaler,
    Part* part,
    StringSynthPart* string_synth) {
  leds_.Init();
  switches_.Init();
  
  settings_ = settings;
  cv_scaler_ = cv_scaler;
  part_ = part;
  string_synth_ = string_synth;
  
  if (switches_.pressed_immediate(1)) {
    State* state = settings_->mutable_state();
    if (state->color_blind == 1) {
      state->color_blind = 0; 
    } else {
      state->color_blind = 1; 
    }
    settings_->Save();
  }
  
  part_->set_polyphony(settings_->state().polyphony);
  part_->set_model(static_cast<ResonatorModel>(settings_->state().model));
  string_synth_->set_polyphony(settings_->state().polyphony);
  string_synth_->set_fx(static_cast<FxType>(settings_->state().model));
  mode_ = settings_->state().easter_egg
      ? UI_MODE_EASTER_EGG_INTRO
      : UI_MODE_NORMAL;

        // Inicializace výstupního DAC na PA4 hned při startu UI na klidový střed
  cv_scaler_->adc().write_dc_offset(1001);    
  
}

void Ui::PollServo() { //Primary funktion -točí se Non-stop loop a dorovnává audio DC offset 

// ============================================================================
//  DEFINICE PARAMETRŮ PRO SERVO AUDIO OFFSET CORCT
// ============================================================================
// NEMĚNNÉ MĚŘÍTKO!!!!!! Ideální hodnota voltů na vstupu do ADC, 
static const int16_t cal_offset_main = 552;
// HODNOTA REEL OFFSET FILTROVANÁ Z ADC (Ultra-zprůměrovaný výstup z filtru).
// podle teplotního driftu JFETu a přičítá/odečítá se vůči cal_offset_main.
static float ultra_filtered_adc = 552.0f; ///////////////gggggggggggggggggggggggggggggggggggg
// Výstup z motoru (navijáku) pro  'DAC'vystup , není propojena s hodnotou rozdílu z ADC výpočtu 
static float auto_dac_main = 1001.0f; 

//float out4 = 1001.0f; // výstup do DAC "statistic" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!§

// VNITŘNÍ MOTOR NAVIJÁKU: Pamatuje si aktuální plynulou rychlost přitahování.
// Tohle je ten klíč – rychlost se nemění skokem, ale plynule dotéká!
static float rychlost_navijaku = 0.7f;
//static float smoothed_dac_start = 0.0f;   // Obyčejný startovní vyhlazovací registr
// true = hardware drží startovací stůl, false = start je odpojen, jede čistá lupa
//static bool servo_startup_gate = true; 
//přichozí parametry ADC "horní a spodní špička"
static float peak_positive = 552.0f;
static float peak_negative = 552.0f;
// Detektor nové hrany multiplexeru (používá stávající last_read bez nových názvů!)
  static int32_t minuly_read = -1;
  int32_t aktualni_read = cv_scaler_->adc().last_read(); //qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq
// ============================================================================
//  PŘESNÉ KOEFICIENTY PRO 4x BIQUAD KASKÁDU (Celkové Fp = 1.4625Hz @ 48 dB/oct)
// ============================================================================
// Tyto koeficienty jsou upravené tak, aby po zařazení 4 stupňů za sebou
// byl výsledný pokles -3dB přesně na frekvenci 1.4625 Hz.
const float b0 = 0.000047f;
const float b1 = 0.000094f;
const float b2 = 0.000047f;
const float a1 = -1.980562f;
const float a2 = 0.980750f;
// --- PAMĚŤOVÉ REGISTRY HISTORIE PRO VŠECHNY 4 STUPNĚ ---
static float biquad1_x1 = 1001.0f, biquad1_x2 = 1001.0f, biquad1_y1 = 1001.0f, biquad1_y2 = 1001.0f;
static float biquad2_x1 = 1001.0f, biquad2_x2 = 1001.0f, biquad2_y1 = 1001.0f, biquad2_y2 = 1001.0f;
static float biquad3_x1 = 1001.0f, biquad3_x2 = 1001.0f, biquad3_y1 = 1001.0f, biquad3_y2 = 1001.0f;
static float biquad4_x1 = 1001.0f, biquad4_x2 = 1001.0f, biquad4_y1 = 1001.0f, biquad4_y2 = 1001.0f;

if (aktualni_read == ADC_CHANNEL_ANALYZE_AUDIO_5 && minuly_read != ADC_CHANNEL_ANALYZE_AUDIO_5) {
// SPRÁVNĚ (čte filtrovanou hodnotu z cv_scaler):
uint32_t raw_sample = cv_scaler_->raw_analyze_audio();
//uint32_t raw_sample = cv_scaler_->adc().raw_analyze_audio();  //obejde skyler !
//ultra_filtered_adc = raw_sample;
//ultra_filtered_adc = static_cast<float>(raw_sample);
 // Čtení surového vzorku z ADC
float raw_p = static_cast<float>(raw_sample);
    if (raw_p > 4095.0f) raw_p = 4095.0f;
    if (raw_p < 0.0f)    raw_p = 0.0f;

///////////PEAK-TO-PEAK TRACKING Symetri oder (MAKRO DIFFERENCIAL FOLOWER)///////////
  if (raw_p > peak_positive) { 
    peak_positive = raw_p; 
  } else {
      peak_positive -= 0.05f * (peak_positive - raw_p); // top peak + in 16bit
  }
  if (raw_p < peak_negative) { 
    peak_negative = raw_p; 
  } else {
    peak_negative += 0.05f * (raw_p - peak_negative); // top peak - in 16bit
  }
  // Bezpečnostní mantinely plováků
    if (peak_positive > 4095.0f) peak_positive = 4095.0f;
    if (peak_negative < 0.0f)    peak_negative = 0.0f;
  //DYNAMICKÝ STŘED: Přesná polovina napěťového pásma signálu za všech podmínek
  //ultra_filtered_adc = (peak_positive + peak_negative) / 2.0f;
  ultra_filtered_adc = (peak_positive + peak_negative) * 0.5f;
  //////////////////////////////////////the end" PEAK-TO-PEAK TRACKING Symetri/////////

  //ODPOJENÍ STARTU: Jakmile tracker poprvé spočítal realitu, startovní napětí vypínáme
  //if (servo_startup_gate) {
   // servo_startup_gate = false;
  //}

  // ==========================================================================
  // SPOLEČNÝ VÝPOČET ROZDÍLU OFFSETU PRO VŠECHNY LUPY""
  // ==========================================================================
  // ---  ADC (Měřítko vstupu) ---
    // 1. Zjistíme pouze směr odchylky (Zda jsme nad nebo pod)
    bool realita_je_vysoko = (ultra_filtered_adc > static_cast<float>(cal_offset_main));
    // 1. Výpočet absolutní odchylky (Zde vypadne čisté kladné číslo)
    float odchylka = realita_je_vysoko ? (ultra_filtered_adc - static_cast<float>(cal_offset_main)) 
                                   : (static_cast<float>(cal_offset_main) - ultra_filtered_adc);

// 2. Plynulé určení cílové rychlosti podle velikosti chyby
   // float cilova_rychlost = odchylka * 0.1f; // uprava hodnot pro naviják !!! 
   // if (cilova_rychlost < 0.05f) {
   // cilova_rychlost = 0.05f; // Pojistka proti záseku
   // }

     // 5. PLYNULÝ NAVIJÁK
    float cilova_rychlost = odchylka * 0.15f;
    if (cilova_rychlost < 0.2f) cilova_rychlost = 0.2f;
    if (cilova_rychlost > 3.0f) cilova_rychlost = 3.0f;

// 3. Exponenciální setrvačnost navijáku (Zrychlování a brždění)
    if (rychlost_navijaku < cilova_rychlost) {
    rychlost_navijaku *= 0.7f; 
    if (rychlost_navijaku > cilova_rychlost) rychlost_navijaku = cilova_rychlost;
    } else if (rychlost_navijaku > cilova_rychlost) {
    rychlost_navijaku *= 0.3f; 
    if (rychlost_navijaku < cilova_rychlost) rychlost_navijaku = cilova_rychlost;
    }
    // 4. PROVEDENÍ KROKU
    // Do DAC se přičítá/odčítá vyhlazená rychlost, nikoliv odchylka z ADC!
    if (odchylka > 0.8f) {  //pojistka testu 
    if (realita_je_vysoko) {
        auto_dac_main -= rychlost_navijaku;
    } else {
        auto_dac_main += rychlost_navijaku;
    }
 }
    // 5. HARDWAROVÁ POJISTKA (Anti-windup)
    if (auto_dac_main > 4095.0f) auto_dac_main = 4095.0f;
    if (auto_dac_main < 0.0f)    auto_dac_main = 0.0f;

  // ==========================================================================
  //  SPOLEČNÝ 4-STUPEŇ BESSEL KASKÁDY (48 dB/oct @ Fp = 1.4625Hz) - HLAVNÍ ŠTÍT
  // ==========================================================================
  // Hodnota z přepínače prochází kaskádou 4 biquadů za sebou.
  // Odstraní VF zvlnění a veškerý přechodový šum těsně před výstupem z multiplexeru.

  // 1. STUPEŇ
  float out1 = b0 * auto_dac_main + b1 * biquad1_x1 + b2 * biquad1_x2 - a1 * biquad1_y1 - a2 * biquad1_y2;
  biquad1_x2 = biquad1_x1; biquad1_x1 = auto_dac_main; biquad1_y2 = biquad1_y1; biquad1_y1 = out1;

  // 2. STUPEŇ
  float out2 = b0 * out1 + b1 * biquad2_x1 + b2 * biquad2_x2 - a1 * biquad2_y1 - a2 * biquad2_y2;
  biquad2_x2 = biquad2_x1; biquad2_x1 = out1; biquad2_y2 = biquad2_y1; biquad2_y1 = out2;

  // 3. STUPEŇ
  float out3 = b0 * out2 + b1 * biquad3_x1 + b2 * biquad3_x2 - a1 * biquad3_y1 - a2 * biquad3_y2;
  biquad3_x2 = biquad3_x1; biquad3_x1 = out2; biquad3_y2 = biquad3_y1; biquad3_y1 = out3;

  // 4. STUPEŇ (Konečný, dokonale čistý stejnosměrný parametr)
  float out4 = b0 * out3 + b1 * biquad4_x1 + b2 * biquad4_x2 - a1 * biquad4_y1 - a2 * biquad4_y2;
  biquad4_x2 = biquad4_x1; biquad4_x1 = out3; biquad4_y2 = biquad4_y1; biquad4_y1 = out4;

 // out4 += 0.1f * (auto_dac_main - out4);

  if (out4 > 4095.0f) out4 = 4095.0f;
  if (out4 < 0.0f) out4 = 0.0f;

  minuly_read = aktualni_read;

//  DAC_SetChannel1Data(DAC_Align_12b_R, static_cast<uint16_t>(out4));
   DAC_SetChannel1Data(DAC_Align_12b_R, static_cast<uint16_t>(out4));
}

void Ui::SaveState() {
  settings_->mutable_state()->polyphony = part_->polyphony();
  settings_->mutable_state()->model = part_->model();
  settings_->Save();
}

void Ui::AnimateEasterEggLeds() {
  mode_ = settings_->state().easter_egg
      ? UI_MODE_EASTER_EGG_INTRO
      : UI_MODE_EASTER_EGG_OUTRO;
}

void Ui::Poll() { //Sekundary funktions 
  // 1kHz.
  system_clock.Tick();
  switches_.Debounce();
  
  for (uint8_t i = 0; i < kNumSwitches; ++i) {
    if (switches_.just_pressed(i)) {
      queue_.AddEvent(CONTROL_SWITCH, i, 0);
      press_time_[i] = system_clock.milliseconds();
    }
    if (switches_.pressed(i) && press_time_[i] != 0) {
      int32_t pressed_time = system_clock.milliseconds() - press_time_[i];
      if (pressed_time > kLongPressDuration) {
        queue_.AddEvent(CONTROL_SWITCH, i, pressed_time);
        press_time_[i] = 0;
      }
    }
    if (switches_.released(i) && press_time_[i] != 0) {
      queue_.AddEvent(
          CONTROL_SWITCH,
          i,
          system_clock.milliseconds() - press_time_[i] + 1);
      press_time_[i] = 0;
    }
  }
  
  bool blink = (system_clock.milliseconds() & 127) > 64;
  bool slow_blink = (system_clock.milliseconds() & 255) > 128;
  switch (mode_) {
    case UI_MODE_NORMAL:
      {
        uint8_t pwm_counter = system_clock.milliseconds() & 15;
        uint8_t triangle = (system_clock.milliseconds() >> 5) & 31;
        triangle = triangle < 16 ? triangle : 31 - triangle;

        if (settings_->state().color_blind == 1) {
          uint8_t mode_red_brightness[] = {
            0, 15, 1,
            0, triangle, uint8_t(triangle >> 3)
          };
          uint8_t mode_green_brightness[] = {
            4, 15, 0, 
            uint8_t(triangle >> 1), triangle, 0,
          };
          
          uint8_t poly_counter = (system_clock.milliseconds() >> 7) % 12;
          uint8_t poly_brightness = (poly_counter >> 1) < part_->polyphony() &&
                (poly_counter & 1);
          uint8_t poly_red_brightness = part_->polyphony() >= 2
              ? 8 + 8 * poly_brightness
              : 0;
          uint8_t poly_green_brightness = part_->polyphony() <= 3
              ? 8 + 8 * poly_brightness
              : 0;
          if (part_->polyphony() == 1 || part_->polyphony() == 4) {
            poly_red_brightness >>= 3;
            poly_green_brightness >>= 2;
          }
          leds_.set(
              0,
              pwm_counter < poly_red_brightness,
              pwm_counter < poly_green_brightness);
          leds_.set(
              1,
              pwm_counter < mode_red_brightness[part_->model()],
              pwm_counter < mode_green_brightness[part_->model()]);
        } else {
          leds_.set(0, part_->polyphony() >= 2, part_->polyphony() <= 2);
          leds_.set(1, part_->model() >= 1, part_->model() <= 1);
          // Fancy modes!
          if (part_->polyphony() == 3) {
            leds_.set(0, true, pwm_counter < triangle);
          }
          if (part_->model() >= 3) {
            bool led_1 = part_->model() >= 4 && pwm_counter < triangle;
            bool led_2 = part_->model() <= 4 && pwm_counter < triangle;
            leds_.set(1, led_1, led_2);
          }
        }
        ++strumming_flag_interval_;
        if (strumming_flag_counter_) {
          --strumming_flag_counter_;
          leds_.set(0, false, false);
        }
      }
      break;
    
    case UI_MODE_CALIBRATION_C1:
      leds_.set(0, blink, blink);
      leds_.set(1, false, false);
      break;

    case UI_MODE_CALIBRATION_C3:
      leds_.set(0, false, false);
      leds_.set(1, blink, blink);
      break;

    case UI_MODE_CALIBRATION_LOW:
      leds_.set(0, slow_blink, 0);
      leds_.set(1, slow_blink, 0);
      break;

    case UI_MODE_CALIBRATION_HIGH:
      leds_.set(0, false, slow_blink);
      leds_.set(1, false, slow_blink);
      break;
    
    case UI_MODE_EASTER_EGG_INTRO:
      {
        uint8_t pwm_counter = system_clock.milliseconds() & 15;
        uint8_t triangle_1 = (system_clock.milliseconds() / 7) & 31;
        uint8_t triangle_2 = (system_clock.milliseconds() / 17) & 31;
        triangle_1 = triangle_1 < 16 ? triangle_1 : 31 - triangle_1;
        triangle_2 = triangle_2 < 16 ? triangle_2 : 31 - triangle_2;
        leds_.set(
            0,
            triangle_1 > pwm_counter,
            triangle_2 > pwm_counter);
        leds_.set(
            1,
            triangle_2 > pwm_counter,
            triangle_1 > pwm_counter);
      }
      break;

    case UI_MODE_EASTER_EGG_OUTRO:
      {
        uint8_t pwm_counter = 7;
        uint8_t triangle_1 = (system_clock.milliseconds() / 9) & 31;
        uint8_t triangle_2 = (system_clock.milliseconds() / 13) & 31;
        triangle_1 = triangle_1 < 16 ? triangle_1 : 31 - triangle_1;
        triangle_2 = triangle_2 < 16 ? triangle_2 : 31 - triangle_2;
        leds_.set(0, triangle_1 < pwm_counter, triangle_1 > pwm_counter);
        leds_.set(1, triangle_2 > pwm_counter, triangle_2 < pwm_counter);
      }
      break;
    
    case UI_MODE_PANIC:
      leds_.set(0, blink, false);
      leds_.set(1, blink, false);
      break;
  }
  leds_.Write();
}

void Ui::FlushEvents() {
  queue_.Flush();
}

void Ui::OnSwitchPressed(const Event& e) {

}

void Ui::OnSwitchReleased(const Event& e) {
  // Check if the other switch is still pressed.
  if (switches_.pressed(1 - e.control_id)) {
    if (mode_ == UI_MODE_CALIBRATION_C1) {
      StartNormalizationCalibration();
    } else {
      StartCalibration();
    }
    press_time_[0] = press_time_[1] = 0;
    return;
  }
  
  switch (e.control_id) {
    case 0:
      if (e.data >= kLongPressDuration) {
        if (cv_scaler_->easter_egg()) {
          settings_->ToggleEasterEgg();
          AnimateEasterEggLeds();
        } else {
          part_->set_polyphony(3);
          string_synth_->set_polyphony(3);
        }
        SaveState();
      } else {
        switch (mode_) {
          case UI_MODE_CALIBRATION_C1:
            CalibrateC1();
            break;
          case UI_MODE_CALIBRATION_C3:
            CalibrateC3();
            break;
          case UI_MODE_CALIBRATION_LOW:
            CalibrateLow();
            break;
          case UI_MODE_CALIBRATION_HIGH:
            CalibrateHigh();
            break;
          default:
            {
              int32_t polyphony = part_->polyphony();
              if (polyphony == 3) {
                polyphony = 2;
              }
              polyphony <<= 1;
              if (polyphony > 4) {
                polyphony = 1;
              }
              part_->set_polyphony(polyphony);
              string_synth_->set_polyphony(polyphony);
              SaveState();
            }
            break;
          }
      }
      break;
    
    case 1:
      if (e.data >= kLongPressDuration) {
        if (cv_scaler_->easter_egg()) {
          settings_->ToggleEasterEgg();
          AnimateEasterEggLeds();
        } else {
          int32_t model = part_->model();
          if (model >= 3) {
            model -= 3;
          } else {
            model += 3;
          }
          part_->set_model(static_cast<ResonatorModel>(model));
          string_synth_->set_fx(static_cast<FxType>(model));
        }
      } else {
        int32_t model = part_->model();
        if (model >= 3) {
          model -= 3;
        } else {
          model = (model + 1) % 3;
        }
        part_->set_model(static_cast<ResonatorModel>(model));
        string_synth_->set_fx(static_cast<FxType>(model));
      }
      SaveState();
      break;
    
    default:
      break;
  }
}

void Ui::StartCalibration() {
  mode_ = UI_MODE_CALIBRATION_C1;
}

void Ui::CalibrateC1() {
  cv_scaler_->CalibrateC1();
  cv_scaler_->CalibrateOffsets();
  mode_ = UI_MODE_CALIBRATION_C3;
}

void Ui::CalibrateC3() {
  bool success = cv_scaler_->CalibrateC3();
  if (success) {
    settings_->Save();
    mode_ = UI_MODE_NORMAL;
  } else {
    mode_ = UI_MODE_PANIC;
  }
}

void Ui::StartNormalizationCalibration() {
  cv_scaler_->StartNormalizationCalibration();
  mode_ = UI_MODE_CALIBRATION_LOW;
}

void Ui::CalibrateLow() {
  cv_scaler_->CalibrateLow();
  mode_ = UI_MODE_CALIBRATION_HIGH;
}

void Ui::CalibrateHigh() {
  bool success = cv_scaler_->CalibrateHigh();
  if (success) {
    settings_->Save();
    mode_ = UI_MODE_NORMAL;
  } else {
    mode_ = UI_MODE_PANIC;
  }
}

void Ui::DoEvents() {
  while (queue_.available()) {
    Event e = queue_.PullEvent();
    if (e.control_type == CONTROL_SWITCH) {
      if (e.data == 0) {
        OnSwitchPressed(e);
      } else {
        OnSwitchReleased(e);
      }
    }
  }
  if (queue_.idle_time() > 800 && mode_ == UI_MODE_PANIC) {
    mode_ = UI_MODE_NORMAL;
  }
  if (mode_ == UI_MODE_EASTER_EGG_INTRO || mode_ == UI_MODE_EASTER_EGG_OUTRO) {
    if (queue_.idle_time() > 3000) {
      mode_ = UI_MODE_NORMAL;
      queue_.Touch();
    }
  } else if (queue_.idle_time() > 1000) {
    queue_.Touch();
  }
}

uint8_t Ui::HandleFactoryTestingRequest(uint8_t command) {
  uint8_t argument = command & 0x1f;
  command = command >> 5;
  uint8_t reply = 0;
  switch (command) {
    case FACTORY_TESTING_READ_POT:
    case FACTORY_TESTING_READ_CV:
      reply = cv_scaler_->adc_value(argument);
      break;
    
    case FACTORY_TESTING_READ_NORMALIZATION:
      reply = cv_scaler_->normalization(argument);
      break;      
    
    case FACTORY_TESTING_READ_GATE:
      reply = argument == 2
          ? cv_scaler_->gate_value()
          : switches_.pressed(argument);
      break;
      
    case FACTORY_TESTING_SET_BYPASS:
      part_->set_bypass(argument);
      break;
      
    case FACTORY_TESTING_CALIBRATE:
      {
        switch (argument) {
          case 0:
            StartCalibration();
            break;
          
          case 1:
            CalibrateC1();
            break;
          
          case 2:
            CalibrateC3();
            break;
          
          case 3:
            StartNormalizationCalibration();
            break;

          case 4:
            CalibrateLow();
            break;
          
          case 5:
            CalibrateHigh();
            queue_.Touch();
            break;
        }
      }
      break;
  }
  return reply;
}

}  // namespace rings
