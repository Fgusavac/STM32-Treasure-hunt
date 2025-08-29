# STM32-Treasure-hunt
An interactive game using button interrupts and USART prompts. Correct inputs reveal the treasure map on an LED matrix with Pirates of the Caribbean theme segments, while mistakes reset progress. Completion unlocks the full theme.


### 📖 Overview  
This module forms the **Treasure Map & Music** component of *Pirates of Engineering*.  
Players interact through **two external buttons**, responding to prompts delivered over **USART**. Correct responses reveal sections of a treasure map on the LED matrix and play segments of the *Pirates of the Caribbean* theme.  

- ✅ Correct input → map progresses + partial theme segment plays  
- ❌ Incorrect input → system resets, LED matrix clears, and no music plays  
- 🎶 Suspense built by splitting the song: one-third plays during progression, with the **full theme unlocked upon final completion**  

---

### 🔌 Hardware Setup  

#### Buttons  
Two input buttons (debounced pull-up resistors):  
- **PA0** → Button 1  
- **PB1** → Button 2  

Wiring:  
3.3V+ → 10kΩ Resistor → Signal Input (PA0 / PB1) → Button → GND


#### LED Matrix (MAX7219)  
- VCC → 5V  
- GND → GND  
- DIN → PA7  
- CLK → PA5  
- CS  → PA4  

#### Piezo Buzzer  
- Positive → PA8 (PWM output via TIM1)  
- Negative → GND  

---

### ⚙️ Peripherals & GPIO Usage  

| Peripheral   | Purpose                            | Notes                                |
|--------------|------------------------------------|--------------------------------------|
| **SPI1**     | Communicates with MAX7219 LED matrix | Patterns defined in `LED_MATRIX.c`   |
| **TIM1**     | Generates PWM for music playback   | Controls buzzer frequency            |
| **USART1**   | Serial messages for prompts/debug  | Baud: 115200                         |
| **GPIO PA0** | Button 1 (interrupt)               | Rising edge detection                |
| **GPIO PB1** | Button 2 (interrupt)               | Rising edge detection                |

---

### 🛠️ Software Structure  

- **main.c / main.h** → Entry point, HAL init for GPIO, SPI, USART, and PWM  
- **GPIO.c / GPIO.h** → Button interrupts and LED handling  
- **serial.c / serial.h** → USART communication for prompts/debugging  
- **LED_MATRIX.c / LED_MATRIX.h** → MAX7219 setup and map display functions  
- **song.c / song.h** → Note arrays, durations, and playback functions  

---

### 🎮 Game Flow  

1. USART prompt instructs player to press one of two buttons  
2. If **correct button** pressed:  
   - Reveal map segment on LED matrix  
   - Play next segment of theme music  
3. If **incorrect button** pressed:  
   - Reset to initial state  
   - Clear LED matrix  
   - No audio played  

Progress continues until all segments are revealed and the **full theme** plays on game completion.  

---

### 🧪 Testing  

- **Buttons**: Verified via pull-up resistor LED test + serial debug messages on rising edge  
- **Buzzer**: Tested standalone, then integrated with `play_tone()` function and note arrays  
- **LED Matrix**: Confirmed via SPI communication with predefined test patterns  
- **USART**: Verified prompts and feedback messages at 115200 baud  

---
