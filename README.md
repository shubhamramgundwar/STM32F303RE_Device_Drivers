# STM32F303RE_Device_Drivers
A comprehensive collection of low-level device drivers developed for STM32Fx series microcontrollers. This repository provides efficient, lightweight, and modular C code implementations that interact directly with STM32 hardware peripherals—perfect for building embedded applications from scratch without relying on HAL or other abstraction layers.

📌 Features
- 🧠 Bare-metal driver development
- ⚙️ Direct register-level programming
- 🚀 Optimized for performance and minimal overhead
- 🔌 Supports key peripherals (GPIO, USART, SPI, I2C, TIM, EXTI, ADC, etc.)
- 🎯 Designed for STM32F0, STM32F1, STM32F4 series (customizable for others)
- 📚 Clear and structured codebase for easy portability and scalability

📂 Repository Structure
├── Drivers/
│   ├── Inc/            # Header files (Peripheral Register Definitions, APIs)
│   └── Src/            # Source files (Driver implementations)
├── Startup/            # Startup files for various STM32Fx MCUs
├── Src/                # Sample applications showcasing driver usage
├── Docs/               # Additional documentation and references
└── README.md           # This file

🧰 Getting Started
Prerequisites
- STM32Fx microcontroller (STM32F103, STM32F407, etc.)
- ARM toolchain (e.g. GCC ARM Embedded)
- Debugger and flashing tool (ST-Link, OpenOCD)
- IDE (STM32CubeIDE, Keil, or Visual Studio Code with Cortex-Debug plugin)
  
Building & Flashing
- Clone the repository
git clone https://github.com/your-username/stm32fx-device-drivers.git
- Open your preferred IDE and import the project
- Compile the project and flash it to your STM32 board

📖 Documentation
Detailed peripheral explanations and usage guides are available in the Docs/ folder, including:
- Reference manual
- Datasheets
- Microcontroller schematics

🙌 Contribution
Feel free to fork and open pull requests to improve or expand the drivers. Here’s how you can contribute:
- Add support for new STM32Fx series
- Fix bugs or improve performance
- Write usage examples and documentation
- Report issues or request features

📞 Contact
Maintainer: Shubham Ramgundwar
GitHub: [https://github.com/shubhamramgundwar]
Email: [shubhamr.ramgundwar@gmail.com]
