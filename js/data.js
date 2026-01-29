const rolesData = [
  {
    id: "rtl-design",
    title: "RTL Design Engineer",
    domain: "VLSI",
    level: "Core VLSI",
    salary: "₹6 LPA – ₹35+ LPA",
    companies: "Qualcomm, Intel, AMD, NVIDIA, Samsung, Synopsys",
    shortDescription: "Design digital circuits using Verilog/SystemVerilog and build micro-architectures for chips.",
    roadmap: {
      basic: {
        fundamentals: [
          "Digital Electronics (Combinational + Sequential)",
          "Number Systems, Boolean Algebra, K-Maps",
          "FSM design, Registers, Counters",
          "Timing basics: Setup/Hold, Clock, Reset",
          "Basic Computer Architecture (ALU, Register File)"
        ],
        skills: [
          "Verilog HDL basics",
          "Writing synthesizable RTL",
          "Coding style: always_ff, always_comb",
          "Testbench basics",
          "Basics of synthesis concepts"
        ],
        tools: [
          "Vivado / Quartus (for practice)",
          "ModelSim / QuestaSim (simulation)",
          "GTKWave (waveform)",
          "VS Code + Verilog extensions"
        ],
        projects: [
          "Design 4-bit/8-bit ALU",
          "UART TX/RX RTL",
          "FIFO design (sync)",
          "Simple RISC CPU (single-cycle)",
          "SPI Controller RTL"
        ],
        others: [
          "Learn good reset strategy (sync/async)",
          "Practice RTL coding on EDAPlayground",
          "Start GitHub portfolio for RTL projects"
        ]
      },
      intermediate: {
        fundamentals: [
          "Pipelining concepts",
          "Cache basics, Memory mapped IO",
          "Clock Domain Crossing (CDC) basics",
          "Bus protocols: AXI-lite basics",
          "Synthesis constraints overview"
        ],
        skills: [
          "SystemVerilog RTL coding",
          "Parameterized modules, generate blocks",
          "Low power concepts: clock gating",
          "Basic lint + CDC understanding",
          "Writing reusable IP blocks"
        ],
        tools: [
          "QuestaSim / Xcelium",
          "Synopsys Design Compiler (industry)",
          "Lint: SpyGlass (industry)",
          "CDC: Questa CDC / SpyGlass CDC"
        ],
        projects: [
          "AXI-lite slave peripheral",
          "Multi-clock FIFO (async FIFO)",
          "Pipelined multiplier",
          "Simple DMA engine",
          "Cache controller (basic)"
        ],
        others: [
          "Learn how to read synthesis reports",
          "Prepare for RTL interview questions",
          "Understand STA basics (timing closure overview)"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced microarchitecture design",
          "Performance/power/area tradeoffs",
          "Advanced CDC/RDC methodology",
          "Low power: UPF concepts",
          "Design for test awareness (DFT-ready RTL)"
        ],
        skills: [
          "Architecting complex SoC blocks",
          "Optimizing RTL for frequency & area",
          "Debugging timing/functional issues with teams",
          "Design reviews and sign-off readiness",
          "Mentoring junior engineers"
        ],
        tools: [
          "Full RTL → Gate flow",
          "UPF/CPF low power tools",
          "Formal verification basics",
          "Regression automation scripts"
        ],
        projects: [
          "High-performance pipelined CPU core",
          "AXI interconnect fabric",
          "Multi-channel DDR controller (learning level)",
          "SoC integration project",
          "Low-power clock gating + power intent demo"
        ],
        others: [
          "Focus on real chip tapeout flow knowledge",
          "Contribute to open-source RISC-V RTL",
          "Learn industry documentation and specs reading"
        ]
      }
    }
  },

  {
    id: "physical-design",
    title: "Physical Design (PD) Engineer",
    domain: "VLSI",
    level: "Back-end VLSI",
    salary: "₹6 LPA – ₹40+ LPA",
    companies: "TSMC, Intel, Qualcomm, Samsung, Cadence, Synopsys",
    shortDescription: "Convert RTL netlist into final chip layout using place & route, timing closure and signoff.",
    roadmap: {
      basic: {
        fundamentals: [
          "CMOS basics",
          "Standard cell concepts",
          "ASIC flow overview (RTL → GDSII)",
          "Timing basics: slack, setup/hold",
          "Floorplanning basics"
        ],
        skills: [
          "Understanding netlist, libraries (.lib)",
          "Basic P&R flow steps",
          "Reading timing reports",
          "Basic TCL scripting"
        ],
        tools: [
          "OpenROAD (open source)",
          "Cadence Innovus (industry)",
          "Synopsys ICC2 (industry)",
          "PrimeTime basics"
        ],
        projects: [
          "Small block P&R using OpenROAD",
          "Timing analysis report reading",
          "Basic floorplan demo",
          "CTS concept demo"
        ],
        others: [
          "Learn standard cell rows & power routing",
          "Understand DRC/LVS meaning"
        ]
      },
      intermediate: {
        fundamentals: [
          "Clock Tree Synthesis (CTS)",
          "Routing congestion & optimization",
          "IR drop and EM basics",
          "Multi-corner multi-mode (MCMM)",
          "ECO flow basics"
        ],
        skills: [
          "Timing closure techniques",
          "Buffer/inverter insertion strategies",
          "Fixing setup/hold violations",
          "TCL automation scripts",
          "Physical optimization"
        ],
        tools: [
          "PrimeTime STA",
          "Innovus / ICC2 advanced commands",
          "Voltus (power integrity)",
          "Calibre (DRC/LVS)"
        ],
        projects: [
          "Run MCMM flow on a design",
          "Fix timing violations using ECO",
          "Create CTS + route reports",
          "Power analysis mini demo"
        ],
        others: [
          "Understand constraints (SDC)",
          "Learn signoff checklist"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced node challenges (7nm/5nm concepts)",
          "Advanced power planning",
          "Chip-level integration & top-level closure",
          "DFM concepts",
          "High-speed design constraints"
        ],
        skills: [
          "Full-chip timing closure ownership",
          "Power-performance-area optimization",
          "Advanced ECO automation",
          "Debugging complex physical issues",
          "Leading tapeout signoff"
        ],
        tools: [
          "Full signoff toolchain",
          "Calibre signoff",
          "PrimeTime advanced analysis",
          "Power signoff tools"
        ],
        projects: [
          "Full chip integration learning project",
          "Advanced STA + ECO closure demo",
          "IR drop improvement study"
        ],
        others: [
          "Learn to coordinate with RTL/DFT/STA teams",
          "Understand tapeout milestones"
        ]
      }
    }
  },

  {
    id: "design-verification",
    title: "Design Verification (DV) Engineer",
    domain: "VLSI",
    level: "Front-end VLSI",
    salary: "₹6 LPA – ₹45+ LPA",
    companies: "NVIDIA, Qualcomm, Intel, AMD, Siemens EDA, Synopsys",
    shortDescription: "Verify RTL design correctness using SystemVerilog, UVM, assertions and coverage.",
    roadmap: {
      basic: {
        fundamentals: [
          "Digital design basics",
          "Verilog/SystemVerilog basics",
          "Testbench architecture",
          "Waveform debugging",
          "Functional simulation flow"
        ],
        skills: [
          "SystemVerilog testbench",
          "Writing tasks/functions",
          "Randomization basics",
          "Assertions (SVA basics)",
          "Coverage basics"
        ],
        tools: [
          "QuestaSim / ModelSim",
          "EDAPlayground",
          "GTKWave",
          "VS Code"
        ],
        projects: [
          "Verify FIFO (directed tests)",
          "Verify UART using SV TB",
          "Simple scoreboard + monitor project",
          "Assertion checks for protocol"
        ],
        others: [
          "Learn debugging with waveforms",
          "Understand bug reporting format"
        ]
      },
      intermediate: {
        fundamentals: [
          "UVM architecture",
          "Transaction-level modeling",
          "Constrained random verification",
          "Functional coverage closure",
          "Regression testing"
        ],
        skills: [
          "Build UVM testbench",
          "Sequences and drivers",
          "Scoreboard + reference model",
          "Coverage-driven verification",
          "Basic scripting for regressions"
        ],
        tools: [
          "Questa UVM",
          "Synopsys VCS (industry)",
          "Coverage tools",
          "Python/Perl scripts for automation"
        ],
        projects: [
          "UVM verification of AXI-lite",
          "Random test generation for ALU",
          "Coverage closure report project",
          "Mini regression setup"
        ],
        others: [
          "Learn debugging failing seeds",
          "Prepare common DV interview topics"
        ]
      },
      expert: {
        fundamentals: [
          "Formal verification concepts",
          "System-level verification",
          "Performance verification",
          "Low power verification",
          "Verification signoff strategy"
        ],
        skills: [
          "Leading verification planning",
          "Advanced assertions and checkers",
          "Building reusable UVM VIP",
          "Debugging complex SoC failures",
          "Mentoring verification teams"
        ],
        tools: [
          "Formal tools (JasperGold, VC Formal)",
          "Advanced simulators",
          "CI/CD for regressions"
        ],
        projects: [
          "SoC-level UVM environment",
          "Protocol checker library",
          "Formal property verification demo"
        ],
        others: [
          "Work closely with architects + RTL team",
          "Learn industry verification methodology"
        ]
      }
    }
  },

  {
    id: "dft-engineer",
    title: "DFT Engineer (Design For Test)",
    domain: "VLSI",
    level: "Test & Manufacturing",
    salary: "₹6 LPA – ₹35+ LPA",
    companies: "Intel, Qualcomm, Synopsys, Tessolve, Samsung",
    shortDescription: "Ensure chip is testable using scan chains, ATPG, BIST and test coverage.",
    roadmap: {
      basic: {
        fundamentals: [
          "Digital logic fundamentals",
          "Fault models (stuck-at)",
          "Scan chain concept",
          "ASIC flow understanding",
          "Basics of testing"
        ],
        skills: [
          "Understanding scan insertion",
          "Test coverage basics",
          "DFT rules awareness",
          "Simple ATPG concepts"
        ],
        tools: [
          "Synopsys DFTMAX (industry)",
          "Tessent (industry)",
          "Basic scripts in TCL"
        ],
        projects: [
          "Scan chain insertion demo (learning)",
          "Simple fault coverage calculation"
        ],
        others: [
          "Learn terminology: scan enable, scan in/out",
          "Understand controllability/observability"
        ]
      },
      intermediate: {
        fundamentals: [
          "ATPG patterns",
          "Compression techniques",
          "MBIST/LBIST basics",
          "Boundary scan (JTAG)",
          "Diagnosis flow"
        ],
        skills: [
          "Generate ATPG patterns",
          "Analyze coverage reports",
          "Debug scan chain issues",
          "Work with PD for DFT fixes"
        ],
        tools: [
          "Tessent ATPG",
          "Synopsys TetraMAX",
          "DFT rule check tools"
        ],
        projects: [
          "ATPG pattern generation demo",
          "MBIST insertion concept project"
        ],
        others: [
          "Learn scan stitching methods",
          "Understand test time reduction"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced DFT architecture",
          "Yield learning",
          "Silicon bring-up test strategy",
          "High quality diagnosis",
          "Production test optimization"
        ],
        skills: [
          "Designing DFT strategy for SoC",
          "Coverage improvement techniques",
          "Debugging silicon test failures",
          "Leading DFT signoff"
        ],
        tools: [
          "Full Tessent flow",
          "ATE pattern handling",
          "Diagnosis tools"
        ],
        projects: [
          "SoC-level DFT planning project",
          "Compression-based ATPG demo"
        ],
        others: [
          "Coordinate with verification & PD teams",
          "Learn manufacturing constraints"
        ]
      }
    }
  },

  {
    id: "analog-layout",
    title: "Analog Layout Engineer",
    domain: "VLSI",
    level: "Analog / Mixed Signal",
    salary: "₹5 LPA – ₹30+ LPA",
    companies: "Texas Instruments, Analog Devices, Qualcomm, Broadcom",
    shortDescription: "Create physical layout for analog blocks like op-amps, PLL, ADC/DAC with matching and DRC/LVS.",
    roadmap: {
      basic: {
        fundamentals: [
          "MOSFET basics",
          "Analog circuits basics (opamp, current mirror)",
          "Layout rules basics",
          "Matching and symmetry concepts",
          "Parasitics basics"
        ],
        skills: [
          "Basic layout drawing",
          "DRC/LVS concepts",
          "Guard ring, shielding",
          "Common centroid layout basics"
        ],
        tools: [
          "Cadence Virtuoso",
          "Calibre DRC/LVS",
          "Assura (some flows)"
        ],
        projects: [
          "Layout of current mirror",
          "Layout of differential pair",
          "Layout of simple opamp"
        ],
        others: [
          "Learn analog layout best practices",
          "Understand process design kits (PDK)"
        ]
      },
      intermediate: {
        fundamentals: [
          "Noise coupling and isolation",
          "IR drop for analog",
          "EM/ESD basics",
          "Analog floorplanning",
          "Parasitic extraction (PEX)"
        ],
        skills: [
          "PEX-aware layout optimization",
          "Matching improvement methods",
          "Routing strategies for analog signals",
          "LVS debugging"
        ],
        tools: [
          "Virtuoso advanced",
          "Calibre xRC / Quantus extraction",
          "Spectre simulation interaction"
        ],
        projects: [
          "Layout of bandgap reference",
          "Layout of ADC block (learning)",
          "PLL layout constraints demo"
        ],
        others: [
          "Work with analog designer closely",
          "Learn tapeout checklist"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced mixed-signal integration",
          "High-speed layout practices",
          "DFM for analog",
          "Analog signoff methodology"
        ],
        skills: [
          "Layout lead for analog macros",
          "Solving complex parasitic issues",
          "Mentoring junior layout engineers",
          "Tapeout ownership"
        ],
        tools: [
          "Signoff PEX tools",
          "Full Cadence flow",
          "Calibre signoff"
        ],
        projects: [
          "Full analog macro layout signoff demo",
          "Mixed-signal top integration project"
        ],
        others: [
          "Focus on silicon debug & iteration learning",
          "Maintain layout quality metrics"
        ]
      }
    }
  },

  {
    id: "sta-engineer",
    title: "STA Engineer (Static Timing Analysis)",
    domain: "VLSI",
    level: "Timing Signoff",
    salary: "₹6 LPA – ₹40+ LPA",
    companies: "Synopsys, Intel, Qualcomm, Samsung, AMD",
    shortDescription: "Analyze and close timing for chips across corners using PrimeTime and constraints.",
    roadmap: {
      basic: {
        fundamentals: [
          "Setup/hold timing concepts",
          "Clock, skew, jitter basics",
          "Timing paths (reg-to-reg, IO paths)",
          "Constraints basics (SDC intro)"
        ],
        skills: [
          "Read timing reports",
          "Basic constraint writing",
          "Understand slack and violations",
          "Basic ECO understanding"
        ],
        tools: [
          "Synopsys PrimeTime (industry)",
          "OpenSTA (practice)",
          "TCL scripting"
        ],
        projects: [
          "Analyze timing on small netlist using OpenSTA",
          "Write simple SDC constraints demo"
        ],
        others: [
          "Understand clock definitions and false paths",
          "Learn common STA interview questions"
        ]
      },
      intermediate: {
        fundamentals: [
          "MCMM analysis",
          "OCV/AOCV/POCV concepts",
          "Clock gating checks",
          "SI effects basics"
        ],
        skills: [
          "Constraint debugging",
          "Timing closure support to PD",
          "Hold fixing strategies",
          "Report automation using TCL"
        ],
        tools: [
          "PrimeTime advanced",
          "Tempus (Cadence)",
          "SI-aware STA tools"
        ],
        projects: [
          "MCMM timing closure demo",
          "Constraint validation script"
        ],
        others: [
          "Learn timing signoff checklist",
          "Understand ECO flow deeply"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced signoff methodologies",
          "Chip-level timing closure",
          "Advanced SI analysis",
          "Power-aware timing"
        ],
        skills: [
          "Leading timing signoff for SoC",
          "Cross-team debugging",
          "Advanced constraints strategy",
          "Mentoring STA engineers"
        ],
        tools: [
          "PrimeTime signoff suite",
          "Advanced analysis features",
          "Custom automation scripts"
        ],
        projects: [
          "Full-chip signoff timing flow demo",
          "SI analysis improvement case study"
        ],
        others: [
          "Learn to handle late-stage ECO closures",
          "Build reusable STA automation library"
        ]
      }
    }
  },

  {
    id: "fpga-design",
    title: "FPGA Design Engineer",
    domain: "FPGA",
    level: "Digital + Prototyping",
    salary: "₹4 LPA – ₹30+ LPA",
    companies: "Xilinx/AMD, Intel FPGA, Qualcomm, ISRO, DRDO",
    shortDescription: "Implement digital designs on FPGA boards using HDL, timing constraints, and hardware debugging.",
    roadmap: {
      basic: {
        fundamentals: [
          "Digital design basics",
          "Verilog/VHDL basics",
          "FPGA architecture basics (LUT, FF, BRAM)",
          "Clocking and reset basics"
        ],
        skills: [
          "Write RTL for FPGA",
          "Simulate using ModelSim",
          "Basic constraints (XDC/SDC)",
          "Basic hardware bring-up"
        ],
        tools: [
          "Xilinx Vivado",
          "Intel Quartus",
          "ModelSim",
          "ILA (Integrated Logic Analyzer)"
        ],
        projects: [
          "Blink LED",
          "UART communication",
          "PWM generator",
          "SPI interface with sensor",
          "Simple VGA display"
        ],
        others: [
          "Buy a beginner FPGA board (Basys3 / DE10)",
          "Practice constraints early"
        ]
      },
      intermediate: {
        fundamentals: [
          "Timing closure on FPGA",
          "Clock domain crossing",
          "AXI stream basics",
          "DSP blocks usage"
        ],
        skills: [
          "Use BRAM/FIFOs",
          "Pipeline design for high frequency",
          "Hardware debugging with ILA",
          "Interface DDR basics (learning)"
        ],
        tools: [
          "Vivado timing analyzer",
          "Quartus timing analyzer",
          "ChipScope/ILA tools"
        ],
        projects: [
          "Image processing pipeline (simple)",
          "FFT implementation",
          "AXI-stream data mover",
          "Soft processor integration (MicroBlaze/Nios II)"
        ],
        others: [
          "Learn throughput/latency calculations",
          "Build GitHub FPGA repo"
        ]
      },
      expert: {
        fundamentals: [
          "High-speed interfaces (PCIe, Ethernet)",
          "Multi-board systems",
          "Hardware/software co-design",
          "FPGA prototyping for ASIC"
        ],
        skills: [
          "Complex system integration",
          "Performance optimization",
          "Timing closure expert level",
          "Team leadership"
        ],
        tools: [
          "Vivado advanced",
          "Protocol analyzers",
          "CI automation for FPGA builds"
        ],
        projects: [
          "Ethernet-based streaming system",
          "PCIe accelerator prototype",
          "Real-time DSP pipeline"
        ],
        others: [
          "Learn about FPGA SoC devices (Zynq)",
          "Contribute to open FPGA projects"
        ]
      }
    }
  },

  {
    id: "communication-engineer",
    title: "Communication Engineer",
    domain: "Communication",
    level: "Wireless / Networking",
    salary: "₹4 LPA – ₹25+ LPA",
    companies: "Qualcomm, Ericsson, Nokia, Samsung, Jio, Airtel",
    shortDescription: "Work on wireless systems like 4G/5G, modulation, coding, RF basics, and communication protocols.",
    roadmap: {
      basic: {
        fundamentals: [
          "Signals & Systems",
          "Fourier Transform basics",
          "Modulation (AM/FM/PM basics)",
          "Digital modulation (BPSK, QPSK)",
          "Probability basics"
        ],
        skills: [
          "MATLAB simulations",
          "Link budget basics",
          "Noise and SNR understanding",
          "Basic channel concepts"
        ],
        tools: [
          "MATLAB",
          "Python (NumPy/Matplotlib)",
          "GNU Radio (optional)"
        ],
        projects: [
          "Simulate BPSK/QPSK BER curves",
          "OFDM basic simulation",
          "Channel noise simulation"
        ],
        others: [
          "Strong math is required",
          "Practice GATE ECE comm topics"
        ]
      },
      intermediate: {
        fundamentals: [
          "OFDM deep understanding",
          "MIMO basics",
          "Coding (Convolutional, Turbo, LDPC intro)",
          "5G NR overview"
        ],
        skills: [
          "Build end-to-end comm chain simulation",
          "Work on equalization",
          "Channel estimation basics",
          "DSP implementation basics"
        ],
        tools: [
          "MATLAB 5G toolbox (optional)",
          "Python comm libraries",
          "SDR basics (USRP/HackRF)"
        ],
        projects: [
          "OFDM with channel estimation",
          "MIMO simulation",
          "5G waveform analysis mini project"
        ],
        others: [
          "Learn networking basics (TCP/IP)",
          "Start SDR practical learning"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced MIMO (Massive MIMO)",
          "Beamforming",
          "Advanced coding/decoding",
          "Wireless standards deep dive"
        ],
        skills: [
          "Algorithm development for 5G/6G",
          "Optimization and performance tuning",
          "System-level simulation",
          "Industry standard compliance"
        ],
        tools: [
          "SystemVue (industry)",
          "MATLAB advanced toolboxes",
          "SDR lab setups"
        ],
        projects: [
          "5G NR PHY chain simulation",
          "Beamforming demo",
          "SDR-based wireless prototype"
        ],
        others: [
          "Research papers reading habit",
          "Good for MS/PhD track too"
        ]
      }
    }
  },

  {
    id: "embedded-engineer",
    title: "Embedded Systems Engineer",
    domain: "Embedded",
    level: "Firmware + Hardware",
    salary: "₹4 LPA – ₹25+ LPA",
    companies: "Bosch, Tata ELXSI, Continental, Qualcomm, STMicro, Texas Instruments",
    shortDescription: "Develop firmware for microcontrollers, drivers, RTOS applications and embedded products.",
    roadmap: {
      basic: {
        fundamentals: [
          "C programming (must)",
          "Microcontroller basics (GPIO, Timers)",
          "UART/SPI/I2C basics",
          "Interrupts basics",
          "Basic electronics"
        ],
        skills: [
          "Embedded C",
          "Register-level programming basics",
          "Basic debugging",
          "Reading datasheets"
        ],
        tools: [
          "Arduino IDE (start)",
          "STM32CubeIDE / Keil",
          "Proteus (optional)",
          "Logic analyzer (optional)"
        ],
        projects: [
          "LED + Button interrupt",
          "Temperature sensor reading",
          "I2C OLED display",
          "UART terminal project",
          "PWM motor control basic"
        ],
        others: [
          "Learn Git",
          "Make project videos for portfolio"
        ]
      },
      intermediate: {
        fundamentals: [
          "RTOS concepts",
          "Memory management basics",
          "Device drivers structure",
          "Communication protocols deeper"
        ],
        skills: [
          "FreeRTOS tasks + queues",
          "Writing HAL drivers",
          "Debugging with GDB",
          "Embedded Linux basics (optional)"
        ],
        tools: [
          "FreeRTOS",
          "STM32 + Debugger (ST-Link)",
          "Segger J-Link",
          "PlatformIO"
        ],
        projects: [
          "RTOS based sensor hub",
          "IoT node using MQTT",
          "Motor control + PID basics",
          "Data logger system"
        ],
        others: [
          "Learn unit testing basics",
          "Learn how to design PCB basics"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced embedded architecture",
          "Safety standards (AUTOSAR basics)",
          "Low power optimization",
          "Performance profiling"
        ],
        skills: [
          "Firmware architecture design",
          "Complex driver development",
          "Optimization for memory & speed",
          "Team leadership"
        ],
        tools: [
          "Embedded Linux (Yocto)",
          "Advanced debugging tools",
          "CI/CD for firmware"
        ],
        projects: [
          "Full product firmware architecture",
          "Custom bootloader",
          "Edge AI on embedded device"
        ],
        others: [
          "Learn documentation + production readiness",
          "Great career growth in Automotive + IoT"
        ]
      }
    }
  },

  {
    id: "pcb-design",
    title: "PCB Design Engineer",
    domain: "Core-ECE",
    level: "Hardware Design",
    salary: "₹3 LPA – ₹20+ LPA",
    companies: "Bosch, Honeywell, L&T, Tata Power, Startups",
    shortDescription: "Design printed circuit boards, schematics, routing, and manufacturing-ready PCB layouts.",
    roadmap: {
      basic: {
        fundamentals: [
          "Basic electronics components",
          "Ohm’s law, KCL/KVL",
          "Power supply basics",
          "Reading circuit diagrams",
          "PCB basics (layers, tracks, vias)"
        ],
        skills: [
          "Schematic capture",
          "Basic routing",
          "Component footprint understanding",
          "ERC/DRC basics"
        ],
        tools: [
          "KiCad (free)",
          "Altium Designer (industry)",
          "EasyEDA (beginner)"
        ],
        projects: [
          "LED driver PCB",
          "Arduino sensor shield",
          "5V/3.3V regulator PCB",
          "Simple amplifier PCB"
        ],
        others: [
          "Learn BOM making",
          "Learn how to order PCB from JLCPCB"
        ]
      },
      intermediate: {
        fundamentals: [
          "Signal integrity basics",
          "Grounding and decoupling",
          "High-speed routing basics",
          "EMI/EMC basics"
        ],
        skills: [
          "Multi-layer PCB design",
          "Impedance controlled routing basics",
          "Power plane design",
          "Design review checklist"
        ],
        tools: [
          "Altium advanced",
          "KiCad advanced",
          "LTSpice simulation"
        ],
        projects: [
          "STM32 board design",
          "Motor driver PCB",
          "High-speed USB routing demo",
          "Power supply PCB (buck converter)"
        ],
        others: [
          "Learn manufacturing files: Gerber, Drill",
          "Learn assembly basics"
        ]
      },
      expert: {
        fundamentals: [
          "High-speed DDR routing concepts",
          "RF PCB layout basics",
          "Advanced EMI debugging",
          "DFM/DFT for PCB"
        ],
        skills: [
          "Complex board architecture",
          "Hardware debugging and bring-up",
          "Design for mass production",
          "Team lead hardware role"
        ],
        tools: [
          "SI/PI tools (HyperLynx)",
          "Oscilloscope + VNA basics",
          "Thermal analysis tools"
        ],
        projects: [
          "High-speed processor board",
          "RF front-end board",
          "Industrial product PCB design"
        ],
        others: [
          "Strong career in hardware startups",
          "Build portfolio with PCB renders + photos"
        ]
      }
    }
  },

  {
    id: "iot-engineer",
    title: "IoT Engineer",
    domain: "Embedded",
    level: "IoT + Cloud + Sensors",
    salary: "₹4 LPA – ₹22+ LPA",
    companies: "Bosch, Siemens, Wipro, TCS, Startups",
    shortDescription: "Build connected devices using sensors, microcontrollers, WiFi/BLE, cloud and dashboards.",
    roadmap: {
      basic: {
        fundamentals: [
          "Microcontroller basics",
          "Sensors basics",
          "WiFi/Bluetooth basics",
          "Networking basics"
        ],
        skills: [
          "ESP32/NodeMCU programming",
          "MQTT basics",
          "HTTP APIs basics",
          "Basic dashboard creation"
        ],
        tools: [
          "Arduino IDE",
          "ESP-IDF (optional)",
          "Node-RED",
          "Firebase (optional)"
        ],
        projects: [
          "Smart temperature monitoring",
          "Home automation switch",
          "IoT gas leak alert system"
        ],
        others: [
          "Learn power saving for IoT",
          "Use GitHub to show IoT projects"
        ]
      },
      intermediate: {
        fundamentals: [
          "Cloud basics",
          "IoT security basics",
          "Device provisioning",
          "OTA updates concept"
        ],
        skills: [
          "AWS IoT / Azure IoT basics",
          "Database basics",
          "Realtime dashboard building",
          "Edge computing basics"
        ],
        tools: [
          "AWS IoT Core",
          "ThingsBoard",
          "Grafana (optional)",
          "MongoDB/Firebase"
        ],
        projects: [
          "Smart energy meter monitoring",
          "IoT + mobile app integration",
          "Industrial sensor dashboard"
        ],
        others: [
          "Learn basic cybersecurity",
          "Understand scalability"
        ]
      },
      expert: {
        fundamentals: [
          "Large-scale IoT architecture",
          "Edge AI concepts",
          "Security hardening",
          "Industrial protocols basics"
        ],
        skills: [
          "End-to-end product deployment",
          "IoT fleet management",
          "Performance and reliability",
          "Leading IoT teams"
        ],
        tools: [
          "Kubernetes for IoT backend (optional)",
          "Device management platforms",
          "Security monitoring tools"
        ],
        projects: [
          "Complete smart city IoT solution",
          "Industrial predictive maintenance system"
        ],
        others: [
          "Very good for startups + product companies",
          "Add real-world deployment proof"
        ]
      }
    }
  },

  {
    id: "power-electronics",
    title: "Power Electronics Engineer",
    domain: "Power",
    level: "Power Systems + Drives",
    salary: "₹4 LPA – ₹18+ LPA",
    companies: "ABB, Siemens, Tata Power, L&T, Schneider",
    shortDescription: "Design and analyze converters, inverters, motor drives and power control systems.",
    roadmap: {
      basic: {
        fundamentals: [
          "Basic circuits",
          "Diodes, MOSFET, IGBT basics",
          "Rectifiers and regulators",
          "Transformers basics"
        ],
        skills: [
          "Simulate converters",
          "Basic control concepts",
          "Component selection basics"
        ],
        tools: [
          "MATLAB/Simulink",
          "PSIM",
          "LTSpice"
        ],
        projects: [
          "Buck converter simulation",
          "Boost converter simulation",
          "Inverter basics simulation"
        ],
        others: [
          "Learn safety precautions",
          "Understand heat dissipation basics"
        ]
      },
      intermediate: {
        fundamentals: [
          "PWM techniques",
          "Closed loop control",
          "Motor drives basics",
          "Power factor correction basics"
        ],
        skills: [
          "Design converter hardware basics",
          "Controller tuning",
          "PCB power layout basics"
        ],
        tools: [
          "Simulink advanced",
          "PSpice",
          "Oscilloscope usage"
        ],
        projects: [
          "Closed-loop buck converter",
          "BLDC motor controller simulation",
          "Solar MPPT demo"
        ],
        others: [
          "Learn EMI/EMC basics",
          "Work on efficiency optimization"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced drives control",
          "Wide bandgap devices (SiC/GaN)",
          "Thermal + reliability analysis",
          "Grid integration concepts"
        ],
        skills: [
          "Industrial product design",
          "Hardware testing and validation",
          "Leadership and architecture"
        ],
        tools: [
          "Hardware lab instruments",
          "Thermal simulation tools",
          "Industrial design tools"
        ],
        projects: [
          "EV inverter design study",
          "High power SMPS prototype"
        ],
        others: [
          "Great role for EV industry growth",
          "Strong demand in power sector"
        ]
      }
    }
  },

  {
    id: "robotics-engineer",
    title: "Robotics Engineer",
    domain: "Robotics",
    level: "Automation + Control",
    salary: "₹4 LPA – ₹30+ LPA",
    companies: "ABB, Fanuc, Boston Dynamics, ISRO, Startups",
    shortDescription: "Build robots using sensors, motors, controllers, embedded systems, and ROS.",
    roadmap: {
      basic: {
        fundamentals: [
          "Basic mechanics",
          "DC motors and drivers",
          "Sensors (IMU, ultrasonic)",
          "Basic control systems"
        ],
        skills: [
          "Arduino/ESP32 basics",
          "Motor control",
          "Basic PID control",
          "Python basics"
        ],
        tools: [
          "Arduino IDE",
          "ROS basics (optional)",
          "MATLAB (optional)"
        ],
        projects: [
          "Line follower robot",
          "Obstacle avoidance robot",
          "Bluetooth controlled car"
        ],
        others: [
          "Learn debugging hardware",
          "Document your robot builds"
        ]
      },
      intermediate: {
        fundamentals: [
          "Kinematics basics",
          "ROS topics (publisher/subscriber)",
          "SLAM basics",
          "Path planning basics"
        ],
        skills: [
          "ROS2 usage",
          "Sensor fusion basics",
          "Computer vision basics",
          "Embedded integration"
        ],
        tools: [
          "ROS2",
          "Gazebo simulator",
          "OpenCV",
          "Raspberry Pi"
        ],
        projects: [
          "Autonomous robot with ROS",
          "Camera based object tracking",
          "Simple SLAM demo"
        ],
        others: [
          "Learn Linux basics",
          "Build portfolio videos"
        ]
      },
      expert: {
        fundamentals: [
          "Advanced SLAM",
          "Robot navigation stack",
          "Real-time control systems",
          "Industrial automation"
        ],
        skills: [
          "Full robotics system architecture",
          "Optimization and performance",
          "Team lead robotics projects"
        ],
        tools: [
          "Advanced ROS2",
          "Lidar integration",
          "Real robot deployment tools"
        ],
        projects: [
          "Lidar based mapping robot",
          "Warehouse automation robot"
        ],
        others: [
          "Good for research + product roles",
          "Strong demand in automation"
        ]
      }
    }
  },

  {
    id: "asic-validation",
    title: "ASIC Validation / Post-Silicon Engineer",
    domain: "VLSI",
    level: "Silicon Bring-up",
    salary: "₹6 LPA – ₹35+ LPA",
    companies: "Intel, Qualcomm, NVIDIA, AMD",
    shortDescription: "Validate chip functionality on real silicon, debug issues and ensure production readiness.",
    roadmap: {
      basic: {
        fundamentals: [
          "Digital systems basics",
          "Interfaces basics (UART/SPI/I2C)",
          "Lab equipment basics",
          "Debugging basics"
        ],
        skills: [
          "Python scripting for testing",
          "Basic board bring-up",
          "Reading datasheets"
        ],
        tools: [
          "Oscilloscope",
          "Logic analyzer",
          "Python automation scripts"
        ],
        projects: [
          "Basic silicon test script simulation",
          "Interface validation demo"
        ],
        others: [
          "Learn documentation and reporting",
          "Good for hardware debugging lovers"
        ]
      },
      intermediate: {
        fundamentals: [
          "PCIe/Ethernet basics",
          "Stress testing concepts",
          "Failure analysis basics"
        ],
        skills: [
          "Automation test frameworks",
          "Debugging complex system issues",
          "Working with firmware teams"
        ],
        tools: [
          "JTAG tools",
          "Lab automation setup",
          "Debug probes"
        ],
        projects: [
          "Automated validation suite",
          "Protocol stress test setup"
        ],
        others: [
          "Strong communication skills needed",
          "Great for system-level roles"
        ]
      },
      expert: {
        fundamentals: [
          "Silicon debug strategy",
          "Yield and reliability testing",
          "Production readiness signoff"
        ],
        skills: [
          "Lead bring-up teams",
          "Root cause analysis expertise",
          "Cross-team coordination"
        ],
        tools: [
          "Advanced debug tools",
          "Failure analysis labs"
        ],
        projects: [
          "Full post-silicon validation program",
          "Yield improvement study"
        ],
        others: [
          "Very high impact role",
          "Grows into architect/lead roles"
        ]
      }
    }
  },
/* =========================================================
   Part-2 Roles (12) — SAME FORMAT as your RTL role object
   Copy-Paste directly into your roles array
   ========================================================= */

{
  id: "rf-engineer",
  title: "RF Engineer",
  domain: "RF & Microwave",
  level: "Wireless / RF Systems",
  salary: "₹4 LPA – ₹30+ LPA",
  companies: "Qualcomm, Ericsson, Nokia, Samsung, Keysight, ISRO, DRDO",
  shortDescription:
    "Designs, tests, and optimizes RF circuits/systems for wireless communication (WiFi/5G/IoT/Radar).",
  roadmap: {
    basic: {
      fundamentals: [
        "EM waves basics: frequency, wavelength, propagation",
        "dB, dBm, SNR, Noise Figure (NF) basics",
        "Impedance matching basics",
        "Transmission lines (coax, microstrip)",
        "S-parameters: S11, S21, VSWR, Return Loss",
        "RF blocks: LNA, PA, Mixer, Filter, VCO"
      ],
      skills: [
        "Read RF datasheets and block diagrams",
        "Basic matching network design (L, Pi)",
        "Understand antenna + RF chain connection",
        "Use VNA/Spectrum Analyzer basics",
        "Basic RF measurement reporting"
      ],
      tools: [
        "Keysight ADS (beginner)",
        "MATLAB (RF calculations + plots)",
        "VNA (S11 measurement + calibration basics)",
        "Spectrum Analyzer (signal observation)"
      ],
      projects: [
        "Design and simulate an LC low-pass filter",
        "Measure S11 of a simple antenna and write report",
        "Create a link budget calculator (Excel/Python)",
        "Simulate matching network for 50Ω system",
        "RF connector/cable loss experiment"
      ],
      others: [
        "Learn RF safety + grounding/shielding basics",
        "Understand connector types (SMA, uFL) and cable losses",
        "Practice reading RF system block diagrams"
      ]
    },
    intermediate: {
      fundamentals: [
        "Noise figure cascade and Friis equation",
        "Linearity metrics: P1dB, IP3",
        "RF system budget: gain/noise/linearity budgeting",
        "Modulation basics: QPSK, QAM, OFDM",
        "Spurs/harmonics and spectral mask",
        "RF PCB layout rules (controlled impedance)"
      ],
      skills: [
        "Design stable RF amplifiers and matching networks",
        "Debug RF spurs/harmonics and improve performance",
        "Do sensitivity and EVM measurements (intro)",
        "RF PCB layout + grounding best practices",
        "Correlation between simulation and measurement"
      ],
      tools: [
        "ADS (S-parameter simulation, matching, stability)",
        "CST/HFSS basics (EM validation)",
        "Vector Signal Analyzer (VSA) basics",
        "Python (SCPI automation for lab instruments)"
      ],
      projects: [
        "Design an RF front-end chain: Filter + LNA + Mixer",
        "Build a 2.4 GHz RF test setup and characterize performance",
        "RF PCB layout mini-project with impedance traces",
        "Create automated measurement script using SCPI",
        "Noise figure and gain measurement report"
      ],
      others: [
        "Learn EMI/EMC basics for RF boards",
        "Create repeatable test procedure documents",
        "Understand tradeoffs: gain vs linearity vs power"
      ]
    },
    expert: {
      fundamentals: [
        "RF architecture: direct conversion vs superhet",
        "Multi-band RF design challenges",
        "Advanced stability and oscillation prevention",
        "EMC/EMI compliance and certification testing",
        "System-level RF optimization and trade-offs"
      ],
      skills: [
        "Own full RF subsystem from architecture to validation",
        "Lead RF characterization and production tuning",
        "Root-cause complex RF failures in field",
        "Collaborate with antenna/PD/firmware teams",
        "Drive performance improvements across product lifecycle"
      ],
      tools: [
        "Advanced ADS (co-simulation with EM)",
        "Measurement automation (Python + SCPI)",
        "Production test analysis tools",
        "EMI/EMC lab equipment workflows"
      ],
      projects: [
        "Complete RF transceiver validation plan + execution",
        "EMI/EMC improvement project for a product",
        "Multi-band IoT RF architecture design study",
        "Production RF calibration and tuning strategy",
        "Field issue debug + RCA documentation"
      ],
      others: [
        "Strong documentation + review skills are critical",
        "Learn regulatory standards (ETSI/FCC) basics",
        "Mentor juniors and lead cross-team RF design reviews"
      ]
    }
  }
},

{
  id: "antenna-engineer",
  title: "Antenna Engineer",
  domain: "RF & Antenna",
  level: "Antenna Design & Measurement",
  salary: "₹4 LPA – ₹28+ LPA",
  companies: "Qualcomm, Samsung, Ericsson, Nokia, ISRO, DRDO, HFCL",
  shortDescription:
    "Designs and optimizes antennas for wireless systems (WiFi/5G/IoT/Radar) focusing on gain, bandwidth, efficiency, and patterns.",
  roadmap: {
    basic: {
      fundamentals: [
        "Maxwell basics and radiation mechanism",
        "Antenna parameters: gain, directivity, efficiency",
        "Bandwidth, resonance, polarization",
        "Radiation patterns and beamwidth",
        "Basic antennas: dipole, monopole, patch, Yagi",
        "S11, VSWR, Return Loss meaning"
      ],
      skills: [
        "Patch antenna basic design calculations",
        "Parametric sweeps (length/width/feed position)",
        "Interpret S11 plots and resonance shift",
        "Basic antenna fabrication understanding"
      ],
      tools: [
        "CST Studio / HFSS / FEKO (any one)",
        "MATLAB (analysis + plots)",
        "VNA basics (S11 measurement)"
      ],
      projects: [
        "Design 2.4 GHz patch antenna and simulate S11",
        "Simulate radiation pattern and gain",
        "Compare FR4 vs Rogers antenna performance",
        "Design monopole antenna for IoT band",
        "Write antenna design report (plots + results)"
      ],
      others: [
        "Learn fabrication basics (FR4 vs Rogers)",
        "Keep parameter tracking table for optimization",
        "Understand feed methods: coax/inset/microstrip"
      ]
    },
    intermediate: {
      fundamentals: [
        "Antenna arrays basics and coupling",
        "Bandwidth enhancement techniques",
        "Feeding techniques and matching",
        "MIMO basics and isolation concepts",
        "Enclosure effect on antenna performance"
      ],
      skills: [
        "Antenna optimization with constraints",
        "Fabrication + measurement correlation",
        "Improve impedance matching and bandwidth",
        "Isolation improvement techniques"
      ],
      tools: [
        "CST/HFSS optimization tools",
        "VNA calibration and measurement workflow",
        "Anechoic chamber basics (if available)"
      ],
      projects: [
        "2×2 patch array for WiFi band",
        "Dual-band antenna (2.4/5 GHz)",
        "Improve isolation between two antennas (MIMO)",
        "Antenna tuning inside enclosure model",
        "Measurement vs simulation comparison report"
      ],
      others: [
        "Learn measurement error sources and reduction",
        "Prepare professional antenna test reports",
        "Study practical smartphone/IoT antenna examples"
      ]
    },
    expert: {
      fundamentals: [
        "Advanced MIMO and decoupling networks",
        "mmWave antennas and phased arrays",
        "Automotive radar antenna basics",
        "Co-design with mechanical + PCB constraints",
        "Reliability and production tolerance considerations"
      ],
      skills: [
        "Compact antenna design for products",
        "Mutual coupling reduction methods",
        "Lead antenna validation and certification support",
        "Own full antenna lifecycle from concept to production"
      ],
      tools: [
        "Advanced EM workflow (meshing, solver choice)",
        "Measurement automation + post-processing scripts",
        "Chamber testing and advanced calibration"
      ],
      projects: [
        "4×4 MIMO antenna design + full measurement report",
        "28 GHz mmWave antenna prototype + correlation",
        "Production-ready IoT antenna tuning strategy",
        "Decoupling network design for MIMO",
        "Full antenna spec + validation document"
      ],
      others: [
        "Master trade-offs: size vs bandwidth vs efficiency",
        "Strong communication with product teams",
        "Mentor juniors and lead antenna reviews"
      ]
    }
  }
},

{
  id: "digital-ic-design-low-power",
  title: "Digital IC Design (Low Power)",
  domain: "VLSI",
  level: "Low Power RTL / Architecture",
  salary: "₹6 LPA – ₹40+ LPA",
  companies: "Qualcomm, Intel, AMD, NVIDIA, Apple, Samsung, Synopsys",
  shortDescription:
    "Designs RTL/architecture optimized for low power using clock gating, power gating, DVFS, and power-aware methodologies.",
  roadmap: {
    basic: {
      fundamentals: [
        "CMOS switching power basics",
        "Dynamic power vs leakage power",
        "RTL design (Verilog/SystemVerilog)",
        "Timing basics: setup/hold, clock, reset",
        "Clock gating concept",
        "Power vs Performance tradeoff basics"
      ],
      skills: [
        "Write clean synthesizable RTL",
        "Implement clock enable/clock gating logic",
        "Reduce toggling by coding techniques",
        "Basic power estimation understanding",
        "Read synthesis power reports (intro)"
      ],
      tools: [
        "ModelSim/QuestaSim (simulation)",
        "Vivado/Quartus (practice)",
        "GTKWave",
        "VS Code + SV extensions"
      ],
      projects: [
        "Low-power counter using clock enable",
        "FSM power modes controller (sleep/active)",
        "Clock gating demo module",
        "Low toggle-rate datapath design",
        "Power estimation mini-report"
      ],
      others: [
        "Learn low-power coding guidelines",
        "Understand reset strategy for power modes",
        "Maintain GitHub portfolio for RTL blocks"
      ]
    },
    intermediate: {
      fundamentals: [
        "Power domains and isolation/retention concepts",
        "Power gating basics and switch cells",
        "DVFS concepts",
        "UPF/CPF fundamentals",
        "Multi-clock and CDC considerations"
      ],
      skills: [
        "Create basic power intent (UPF)",
        "Power-aware simulation setup",
        "Optimize RTL for low switching activity",
        "Work with PD/DV teams for low-power signoff",
        "Debug low-power mode issues"
      ],
      tools: [
        "Synopsys Design Compiler / Cadence Genus",
        "UPF flow basics",
        "Lint tools (SpyGlass - optional)",
        "Power-aware simulation tools"
      ],
      projects: [
        "Multi-power domain RTL with UPF",
        "Retention flop and isolation demo",
        "Low-power mode regression tests",
        "Clock gating insertion exploration",
        "Power state machine integration"
      ],
      others: [
        "Learn how to read UPF-related reports",
        "Understand low-power verification challenges",
        "Practice interview questions for low-power design"
      ]
    },
    expert: {
      fundamentals: [
        "Low-power SoC architecture design",
        "Advanced leakage reduction techniques",
        "PPA (Power-Performance-Area) tradeoffs",
        "Silicon bring-up low-power debug",
        "Low-power signoff strategy"
      ],
      skills: [
        "Lead low-power architecture decisions",
        "Drive power closure with cross teams",
        "Debug corner-case failures in power states",
        "Mentor juniors and review RTL for power",
        "Own low-power signoff readiness"
      ],
      tools: [
        "PrimeTime PX / Power analysis tools",
        "Advanced UPF flows",
        "Regression automation scripts",
        "Post-silicon power measurement correlation"
      ],
      projects: [
        "Tapeout-ready low-power IP block",
        "Full-chip power optimization initiative",
        "DVFS design strategy for SoC",
        "Production power characterization report",
        "Low-power bug RCA documentation"
      ],
      others: [
        "Low power needs strong spec reading + discipline",
        "Learn industry best practices from real tapeout flows",
        "Drive reviews and ensure power intent correctness"
      ]
    }
  }
},

{
  id: "mixed-signal-verification",
  title: "Mixed Signal Verification Engineer",
  domain: "VLSI",
  level: "AMS / Mixed-Signal Verification",
  salary: "₹6 LPA – ₹35+ LPA",
  companies: "Qualcomm, Intel, Texas Instruments, Samsung, Cadence, Synopsys",
  shortDescription:
    "Verifies analog + digital interactions (ADC/DAC/PLL/SerDes) using AMS simulation, UVM, checkers, and real-world signal validation.",
  roadmap: {
    basic: {
      fundamentals: [
        "Analog basics: op-amp, filters, ADC/DAC",
        "Digital design + verification basics",
        "Basic interfaces: SPI, I2C, UART",
        "Sampling and quantization basics",
        "Waveform analysis basics"
      ],
      skills: [
        "SystemVerilog basics",
        "Basic testbench writing",
        "Understand stimulus/response checking",
        "Basic debugging using waveforms",
        "Write simple assertions (intro)"
      ],
      tools: [
        "Questa / Xcelium / VCS (any)",
        "GTKWave / SimVision",
        "MATLAB (signal plotting)"
      ],
      projects: [
        "Verify ADC digital interface (SPI) using test vectors",
        "Checker for conversion-ready timing behavior",
        "Basic testbench for DAC control interface",
        "Create waveform-based pass/fail check",
        "Mini regression with multiple tests"
      ],
      others: [
        "Learn bug reporting and tracking format",
        "Understand expected behavior from specs",
        "Build habit of writing clean reusable TB code"
      ]
    },
    intermediate: {
      fundamentals: [
        "AMS modeling concepts",
        "Analog-digital boundary issues",
        "Coverage-driven verification basics",
        "Clocking, jitter and sampling impacts",
        "Scoreboarding and reference modeling"
      ],
      skills: [
        "Build UVM environment for mixed-signal block",
        "SVA assertions for protocol checks",
        "Functional coverage + scoreboard",
        "Create basic analog reference models",
        "Regression automation basics"
      ],
      tools: [
        "Cadence Xcelium AMS / Questa ADMS",
        "Python for regression automation",
        "UVM libraries"
      ],
      projects: [
        "Verify PLL lock detect + control logic",
        "ADC/DAC verification with sampled waveforms",
        "Coverage closure report for mixed-signal block",
        "Build reusable UVM agent for interface",
        "Automated regression with logs summary"
      ],
      others: [
        "Learn to reduce simulation time with smart planning",
        "Collaborate with analog designers for expected behavior",
        "Maintain verification plan document"
      ]
    },
    expert: {
      fundamentals: [
        "Full AMS SoC verification strategy",
        "Corner-case validation and robustness",
        "Correlation with silicon lab measurements",
        "Performance validation (jitter/noise sensitivity)",
        "Verification signoff methodology"
      ],
      skills: [
        "Build scalable AMS regression infrastructure",
        "Debug complex analog/digital interactions",
        "Lead verification planning and signoff closure",
        "Own VIP development and reuse",
        "Mentor team and improve verification quality"
      ],
      tools: [
        "Advanced AMS simulation flows",
        "CI/CD for regressions (Jenkins/GitHub Actions)",
        "Python automation frameworks"
      ],
      projects: [
        "Complete AMS verification environment for SerDes/PLL subsystem",
        "Signoff-level verification plan + closure report",
        "Silicon correlation project (pre vs post silicon)",
        "Reusable VIP for mixed-signal interfaces",
        "Performance stress testing suite"
      ],
      others: [
        "Strong collaboration with analog designers is key",
        "Build reusable infrastructure to save team time",
        "Document everything like real industry signoff"
      ]
    }
  }
},

{
  id: "ams-design-engineer",
  title: "AMS Design Engineer",
  domain: "Analog & Mixed Signal",
  level: "Analog Circuit Design",
  salary: "₹6 LPA – ₹45+ LPA",
  companies: "Texas Instruments, Analog Devices, Qualcomm, Intel, Samsung, NXP",
  shortDescription:
    "Designs analog and mixed-signal circuits like op-amps, LDOs, PLLs, ADCs, DACs with PVT and post-layout signoff.",
  roadmap: {
    basic: {
      fundamentals: [
        "MOSFET operation and biasing",
        "Small-signal analysis basics",
        "Op-amp fundamentals and feedback",
        "Stability basics (phase margin)",
        "SPICE simulation basics",
        "Basic layout awareness (matching, symmetry)"
      ],
      skills: [
        "Schematic capture and simulation setup",
        "DC/AC/Transient analysis",
        "Basic compensation techniques",
        "Interpret gain/phase plots",
        "Write design notes and spec sheets"
      ],
      tools: [
        "Cadence Virtuoso (preferred)",
        "Spectre / HSPICE (industry)",
        "LTspice (practice)"
      ],
      projects: [
        "Design a 2-stage op-amp (spec-based)",
        "Basic bandgap reference exploration",
        "Current mirror design and analysis",
        "LDO block-level simulation",
        "Noise simulation mini-study"
      ],
      others: [
        "Practice hand calculations + verify with SPICE",
        "Maintain PVT corner checklist",
        "Learn to read PDK documentation (intro)"
      ]
    },
    intermediate: {
      fundamentals: [
        "ADC/DAC architectures overview",
        "PLL fundamentals",
        "Noise analysis (thermal/flicker)",
        "PVT corners and Monte Carlo",
        "Mismatch and offset concepts"
      ],
      skills: [
        "Design for corners and robustness",
        "Offset/noise optimization",
        "Post-layout awareness (parasitics impact)",
        "Create testbenches for block verification",
        "Basic silicon bring-up preparation"
      ],
      tools: [
        "Monte Carlo simulation setup",
        "Layout extraction basics (PEX intro)",
        "Virtuoso ADE / Spectre flows"
      ],
      projects: [
        "Design an LDO regulator (with stability check)",
        "Design a SAR ADC block (concept + simulation)",
        "PLL loop filter simulation",
        "Post-layout performance comparison demo",
        "Corner + Monte Carlo report generation"
      ],
      others: [
        "Learn layout review checklist (matching, shielding)",
        "Work with layout engineer for constraints",
        "Document design trade-offs clearly"
      ]
    },
    expert: {
      fundamentals: [
        "High-speed analog blocks overview (SerDes/Clocking)",
        "Low-power analog design strategies",
        "Yield and reliability basics",
        "Tapeout signoff requirements",
        "DFM concepts"
      ],
      skills: [
        "Tapeout readiness and signoff ownership",
        "Silicon bring-up and debug",
        "Mentor juniors and lead block ownership",
        "Drive PPA and yield improvements",
        "Work across teams for integration success"
      ],
      tools: [
        "Full signoff flow (PEX, corners, reliability checks)",
        "Advanced noise and transient analysis",
        "Silicon correlation workflows"
      ],
      projects: [
        "Tapeout-ready analog IP with post-layout signoff report",
        "Silicon debug and performance tuning project",
        "High-accuracy reference design",
        "Low-power analog subsystem design",
        "Reliability improvement case study"
      ],
      others: [
        "Expert analog design needs deep fundamentals",
        "Build strong intuition through repeated simulations",
        "Review specs carefully and document assumptions"
      ]
    }
  }
},

{
  id: "soc-integration-engineer",
  title: "SoC Integration Engineer",
  domain: "VLSI",
  level: "SoC / Integration",
  salary: "₹7 LPA – ₹40+ LPA",
  companies: "Qualcomm, Intel, AMD, NVIDIA, Samsung, MediaTek, Broadcom",
  shortDescription:
    "Integrates multiple IP blocks into a full SoC, manages connectivity, clocks/resets, address maps, and integration debug.",
  roadmap: {
    basic: {
      fundamentals: [
        "SoC architecture overview",
        "Bus protocols: AXI/AHB/APB basics",
        "Register maps and address decoding",
        "Clock/reset basics",
        "Interrupt basics"
      ],
      skills: [
        "RTL integration of small IPs",
        "Top module creation and wiring",
        "Basic wrapper creation for IP blocks",
        "Understand memory map and registers",
        "Basic waveform debug"
      ],
      tools: [
        "Verilog/SystemVerilog",
        "QuestaSim / ModelSim",
        "Python/TCL basics",
        "Git + Linux"
      ],
      projects: [
        "Integrate UART + SPI + Timer into a SoC wrapper",
        "Create a simple address decoder",
        "APB peripheral integration demo",
        "Basic interrupt controller integration",
        "Write memory map documentation"
      ],
      others: [
        "Keep clean documentation of address maps",
        "Learn to read IP integration guides",
        "Practice debugging integration mismatches"
      ]
    },
    intermediate: {
      fundamentals: [
        "Interrupt architecture (GIC basics)",
        "CDC and RDC basics",
        "Integration constraints and build flow",
        "SoC-level reset strategies",
        "Basic synthesis awareness"
      ],
      skills: [
        "CDC/RDC issue identification and fixing",
        "Integration-level debug and waveform analysis",
        "Build scalable top-level connectivity",
        "Integrate interconnect and DMA blocks",
        "Work with DV team for SoC tests"
      ],
      tools: [
        "SpyGlass / Questa CDC (any)",
        "Synthesis tools basics (DC/Genus)",
        "Python automation for build/regressions"
      ],
      projects: [
        "Integrate DMA + AXI interconnect + memory controller",
        "SoC integration testbench for IP connectivity",
        "CDC clean multi-clock subsystem integration",
        "SoC reset/clock tree integration demo",
        "Create integration checklist and signoff report"
      ],
      others: [
        "Strong communication with DV/PD/FW teams",
        "Learn integration bug patterns and fixes",
        "Maintain integration documentation"
      ]
    },
    expert: {
      fundamentals: [
        "Full SoC bring-up planning",
        "Power intent integration (UPF) at SoC level",
        "Performance tuning and QoS basics",
        "SoC signoff readiness checklist",
        "Post-silicon integration debug basics"
      ],
      skills: [
        "Lead integration and resolve cross-team issues",
        "Own integration signoff and release process",
        "Drive schedule and integration stability",
        "Debug complex system-level failures",
        "Mentor integration team"
      ],
      tools: [
        "SoC automation frameworks",
        "Advanced regression/build systems",
        "UPF integration tools",
        "CI tools (Jenkins/GitHub Actions)"
      ],
      projects: [
        "Full SoC integration for tapeout",
        "Integration closure reports and release management",
        "Power-aware SoC integration demo",
        "SoC performance tuning case study",
        "Post-silicon debug and patch workflow"
      ],
      others: [
        "Integration engineers are bridge between all teams",
        "Keep communication strong and document changes",
        "Master specs reading and cross-team coordination"
      ]
    }
  }
},

{
  id: "silicon-cad-eda-automation",
  title: "Silicon CAD / EDA Automation Engineer",
  domain: "VLSI",
  level: "EDA / Automation",
  salary: "₹6 LPA – ₹35+ LPA",
  companies: "Synopsys, Cadence, Siemens EDA, Intel, Qualcomm, Samsung",
  shortDescription:
    "Builds automation scripts and flows for EDA tools (lint/synthesis/STA/PD), report parsing, and regression pipelines.",
  roadmap: {
    basic: {
      fundamentals: [
        "Linux basics (commands, permissions, paths)",
        "EDA flow overview (RTL → Synthesis → STA → PD → Signoff)",
        "Basic scripting concepts",
        "Files/logs parsing basics",
        "Git basics"
      ],
      skills: [
        "Python scripting for automation",
        "TCL scripting basics",
        "Parse logs/reports and summarize results",
        "Write reusable scripts",
        "Basic debugging of automation failures"
      ],
      tools: [
        "Python",
        "Bash",
        "TCL",
        "Git"
      ],
      projects: [
        "Script to parse timing reports and summarize worst paths",
        "Auto organize logs/reports by run name/date",
        "Create CSV summary of multiple tool runs",
        "Build small CLI tool for flow automation",
        "Simple report dashboard (HTML/CSV)"
      ],
      others: [
        "Write clean modular code",
        "Use config files (JSON/YAML) for flexibility",
        "Learn basic software engineering practices"
      ]
    },
    intermediate: {
      fundamentals: [
        "Regression frameworks concepts",
        "Flow dependencies and tool configuration",
        "Data extraction and visualization",
        "Runtime and resource usage basics",
        "Error handling strategies"
      ],
      skills: [
        "Build EDA pipelines (lint → synth → STA)",
        "Create dashboards and trend analysis",
        "Robust automation with retries and logs",
        "Integrate CI systems",
        "Write documentation for flows"
      ],
      tools: [
        "Jenkins / GitHub Actions",
        "CSV/SQLite",
        "Matplotlib (basic plots)"
      ],
      projects: [
        "PPA dashboard generator from multiple runs",
        "Full automation pipeline with email/report outputs",
        "Flow runtime optimization project",
        "Tool log analyzer with error classification",
        "Regression manager script with status summary"
      ],
      others: [
        "Focus on stability: automation should not break easily",
        "Maintain versioned flow releases",
        "Learn team collaboration practices"
      ]
    },
    expert: {
      fundamentals: [
        "Compute farm scheduling basics (LSF/SLURM)",
        "Tool licensing management concepts",
        "Large project flow scaling",
        "Security and access management basics",
        "Enterprise flow standardization"
      ],
      skills: [
        "Optimize runtime and resource usage at scale",
        "Debug complex tool failures/environment issues",
        "Create standardized flows across teams",
        "Lead automation architecture decisions",
        "Mentor juniors and enforce coding standards"
      ],
      tools: [
        "SLURM/LSF basics",
        "Advanced Python frameworks",
        "Container basics (Docker - optional)"
      ],
      projects: [
        "Company-level EDA flow automation framework",
        "Runtime reduction and stability improvement project",
        "Centralized PPA tracking system",
        "Automated signoff report generation pipeline",
        "EDA environment health checker tool"
      ],
      others: [
        "This role impacts productivity and tapeout success",
        "Strong debugging skills are mandatory",
        "Always document flow assumptions and constraints"
      ]
    }
  }
},

{
  id: "test-engineer-ate",
  title: "Test Engineer (ATE)",
  domain: "Semiconductor",
  level: "Production Testing / ATE",
  salary: "₹4 LPA – ₹25+ LPA",
  companies: "Texas Instruments, Intel, Qualcomm, Micron, TSMC, ASE, Amkor",
  shortDescription:
    "Develops and executes automated tests for ICs using ATE platforms, improves yield, reduces test time, and ensures quality.",
  roadmap: {
    basic: {
      fundamentals: [
        "Semiconductor test basics (functional vs parametric)",
        "Datasheet understanding and spec limits",
        "Digital measurement basics",
        "DC parameters basics (voltage/current)",
        "Basic debugging mindset"
      ],
      skills: [
        "Read test specifications and write test steps",
        "Basic lab equipment usage (scope, DMM)",
        "Understand pin-level behavior",
        "Identify failing patterns in outputs",
        "Basic documentation and reporting"
      ],
      tools: [
        "Oscilloscope",
        "Multimeter",
        "Power supply",
        "Basic ATE exposure"
      ],
      projects: [
        "Write test checklist for IC pins and functions",
        "Measure DC parameters and create report",
        "Basic functional test sequence design",
        "Simple failure log analysis (Excel)",
        "Bench vs spec correlation study"
      ],
      others: [
        "Follow ESD handling rules strictly",
        "Learn safety and lab best practices",
        "Maintain test documentation cleanly"
      ]
    },
    intermediate: {
      fundamentals: [
        "DFT basics (scan/BIST overview)",
        "Yield and failure analysis basics",
        "Test coverage and test time concepts",
        "Correlation between bench and ATE",
        "Parametric test methods"
      ],
      skills: [
        "Develop ATE test programs",
        "Analyze test data and identify failure patterns",
        "Improve test flow for stability",
        "Debug hardware + software test issues",
        "Optimize test sequence for time"
      ],
      tools: [
        "Teradyne / Advantest (any)",
        "Python for test data analysis",
        "SQL/CSV for data storage (optional)"
      ],
      projects: [
        "Build ATE program for functional + parametric testing",
        "Yield analysis report from test logs",
        "Reduce test time by optimizing patterns",
        "Failure binning strategy implementation",
        "Correlation report (bench vs ATE)"
      ],
      others: [
        "Learn failure analysis communication",
        "Work with DFT/design teams for coverage improvement",
        "Maintain clear test program version control"
      ]
    },
    expert: {
      fundamentals: [
        "High-volume manufacturing test optimization",
        "RCA methodologies (5-Why, Fishbone)",
        "Production quality metrics",
        "Cost-of-test and throughput",
        "Reliability testing basics"
      ],
      skills: [
        "Reduce test time without losing coverage",
        "Drive yield improvement initiatives",
        "Lead debug and failure analysis",
        "Own test strategy for new product",
        "Mentor juniors and standardize test flows"
      ],
      tools: [
        "Advanced data dashboards",
        "Big-data analysis tools (optional)",
        "Production monitoring systems"
      ],
      projects: [
        "Production test optimization project (time + yield)",
        "Full test strategy for a new chip product",
        "Automated yield dashboard pipeline",
        "Failure mode classification system",
        "Continuous improvement documentation"
      ],
      others: [
        "Test engineers directly impact cost and product success",
        "Strong analysis and communication is required",
        "Learn production constraints and real-world debugging"
      ]
    }
  }
},

{
  id: "cybersecurity-for-embedded",
  title: "Cybersecurity for Embedded Systems",
  domain: "Embedded",
  level: "Embedded Security",
  salary: "₹6 LPA – ₹40+ LPA",
  companies: "Bosch, Continental, Qualcomm, Siemens, Honeywell, Palo Alto Networks",
  shortDescription:
    "Secures embedded systems (IoT/ECUs) using secure coding, cryptography, secure boot, OTA security, and threat modeling.",
  roadmap: {
    basic: {
      fundamentals: [
        "Embedded C basics",
        "Security basics: CIA triad",
        "Common vulnerabilities (buffer overflow, weak auth)",
        "Basic cryptography concepts (AES/RSA/Hash)",
        "Networking basics (TCP/IP)"
      ],
      skills: [
        "Secure coding principles",
        "Basic Linux usage",
        "Understand authentication and passwords",
        "Basic encryption usage (library level)",
        "Logging and debugging securely"
      ],
      tools: [
        "Wireshark (basic)",
        "Linux",
        "Git"
      ],
      projects: [
        "Secure boot concept demo (simulation)",
        "Basic firmware vulnerability checklist",
        "Password hashing demo using SHA",
        "Secure UART communication concept",
        "Network packet inspection using Wireshark"
      ],
      others: [
        "Learn safe handling of keys and secrets",
        "Understand basic threat modeling mindset",
        "Avoid hardcoding credentials in firmware"
      ]
    },
    intermediate: {
      fundamentals: [
        "Threat modeling for IoT/embedded devices",
        "Secure OTA updates",
        "Certificates and TLS basics",
        "Firmware reverse engineering basics",
        "Secure storage basics"
      ],
      skills: [
        "Implement AES/RSA usage using libraries",
        "Secure firmware update pipeline",
        "Firmware hardening techniques",
        "Basic penetration testing mindset",
        "Vulnerability assessment reporting"
      ],
      tools: [
        "Ghidra (intro)",
        "OpenSSL basics",
        "Python automation scripts"
      ],
      projects: [
        "Secure firmware update pipeline prototype",
        "IoT device security assessment mini-project",
        "TLS connection setup for embedded device",
        "Basic secure key storage demo",
        "Firmware analysis report (educational)"
      ],
      others: [
        "Learn secure logging and tamper detection",
        "Follow secure SDLC practices",
        "Understand real-world attack scenarios"
      ]
    },
    expert: {
      fundamentals: [
        "Hardware security: TPM/HSM/TrustZone",
        "Secure architecture design",
        "Side-channel attack awareness",
        "Automotive cybersecurity standards (overview)",
        "Secure manufacturing and provisioning"
      ],
      skills: [
        "Design secure product architecture end-to-end",
        "Lead security audits and compliance",
        "Build secure key provisioning systems",
        "Implement secure boot + attestation",
        "Mentor team and review secure designs"
      ],
      tools: [
        "Security testing toolchain",
        "Threat modeling frameworks",
        "CI pipelines for security scanning"
      ],
      projects: [
        "End-to-end secure embedded product design",
        "Security hardening + penetration testing report",
        "Secure provisioning workflow design",
        "Secure OTA and rollback protection system",
        "Threat model + mitigation documentation"
      ],
      others: [
        "High demand domain with strong growth",
        "Security needs continuous learning",
        "Documentation is as important as implementation"
      ]
    }
  }
},

{
  id: "automotive-electronics-engineer",
  title: "Automotive Electronics Engineer",
  domain: "Automotive",
  level: "ECU / Automotive Embedded",
  salary: "₹4 LPA – ₹30+ LPA",
  companies: "Bosch, Continental, Tata Elxsi, Valeo, Mahindra, Maruti Suzuki R&D",
  shortDescription:
    "Designs and validates automotive ECUs, sensors, and communication systems (CAN/LIN/Ethernet) with safety standards.",
  roadmap: {
    basic: {
      fundamentals: [
        "Automotive ECU basics",
        "Sensors and actuators overview",
        "CAN communication basics",
        "Embedded programming basics",
        "Basic power electronics awareness"
      ],
      skills: [
        "Embedded C",
        "CAN messaging and decoding",
        "Basic debugging using logs",
        "Read automotive wiring diagrams",
        "Basic firmware development workflow"
      ],
      tools: [
        "CANoe/CANalyzer (basic)",
        "Arduino/STM32 (practice)",
        "Oscilloscope basics"
      ],
      projects: [
        "CAN communication demo between two nodes",
        "Simple ECU simulator with sensor input",
        "CAN message logger project",
        "Basic actuator control firmware",
        "Fault detection and LED indicator logic"
      ],
      others: [
        "Learn automotive wiring and connectors basics",
        "Understand ECU boot process basics",
        "Keep documentation clean and structured"
      ]
    },
    intermediate: {
      fundamentals: [
        "AUTOSAR basics",
        "Functional safety intro (ISO 26262)",
        "Diagnostics basics (UDS)",
        "RTOS concepts and scheduling",
        "HIL testing basics (overview)"
      ],
      skills: [
        "Implement diagnostics services (UDS)",
        "ECU integration testing",
        "RTOS task design",
        "CAN-based network debugging",
        "Fault handling and logging"
      ],
      tools: [
        "Vector tools (CANoe/CANalyzer)",
        "FreeRTOS basics",
        "Python log analysis"
      ],
      projects: [
        "Mini ECU project with CAN + UDS diagnostics",
        "ECU firmware with fault handling and logging",
        "RTOS-based sensor acquisition system",
        "HIL simulation concept project",
        "CAN diagnostics request-response demo"
      ],
      others: [
        "Learn how to write test cases",
        "Understand safety requirements documentation",
        "Practice ECU debug scenarios"
      ]
    },
    expert: {
      fundamentals: [
        "ADAS basics and sensor fusion overview",
        "Automotive cybersecurity basics",
        "System-level safety design",
        "Automotive Ethernet basics",
        "Production and validation lifecycle"
      ],
      skills: [
        "Lead ECU architecture and validation planning",
        "Drive safety case documentation",
        "Own complex integration issues",
        "Coordinate cross-functional teams",
        "Mentor junior engineers"
      ],
      tools: [
        "HIL systems (dSPACE/Vector)",
        "Automotive test frameworks",
        "Advanced diagnostic toolchains"
      ],
      projects: [
        "Complete ECU validation pipeline and release readiness",
        "System-level automotive integration project",
        "ADAS ECU subsystem design study",
        "Safety compliance documentation project",
        "Production debug + RCA documentation"
      ],
      others: [
        "Automotive domain needs strong standards knowledge",
        "Documentation quality matters a lot",
        "Be comfortable with cross-team collaboration"
      ]
    }
  }
},

{
  id: "ai-hardware-engineer",
  title: "AI Hardware Engineer",
  domain: "VLSI / AI",
  level: "AI Acceleration / Hardware",
  salary: "₹8 LPA – ₹60+ LPA",
  companies: "NVIDIA, Intel, AMD, Qualcomm, Google, Apple, Samsung",
  shortDescription:
    "Builds hardware acceleration for AI workloads (FPGA/ASIC/GPU optimization) focusing on throughput, latency, and power efficiency.",
  roadmap: {
    basic: {
      fundamentals: [
        "Digital logic basics",
        "Computer architecture basics",
        "Neural network basics (CNN, MLP)",
        "Matrix multiplication concept",
        "Fixed-point vs floating-point basics"
      ],
      skills: [
        "Python basics for AI",
        "Verilog basics",
        "FPGA introduction",
        "Basic profiling understanding",
        "Learn quantization basics (int8 vs fp32)"
      ],
      tools: [
        "Python + PyTorch basics",
        "Verilog simulator",
        "FPGA board (optional)"
      ],
      projects: [
        "Implement MAC unit in Verilog",
        "Small CNN inference on Raspberry Pi",
        "Quantize a model and compare accuracy",
        "Basic systolic array concept simulation",
        "Mini report on latency vs throughput"
      ],
      others: [
        "Learn how AI models map to hardware operations",
        "Maintain GitHub for hardware + AI experiments",
        "Understand compute vs memory bottlenecks"
      ]
    },
    intermediate: {
      fundamentals: [
        "Hardware acceleration concepts",
        "Parallel processing and pipelining",
        "Memory bandwidth bottlenecks basics",
        "On-chip memory hierarchy basics",
        "Compute scheduling concepts"
      ],
      skills: [
        "Inference optimization (quantization/pruning)",
        "FPGA acceleration design flow",
        "Performance estimation and profiling",
        "Basic HLS understanding (optional)",
        "Pipeline and resource tradeoffs"
      ],
      tools: [
        "Xilinx Vivado/Vitis",
        "TensorRT basics",
        "Python profiling tools"
      ],
      projects: [
        "FPGA accelerator for matrix multiply",
        "Edge AI deployment with optimization",
        "Latency measurement + optimization report",
        "Design pipeline for convolution operation",
        "Compare FPGA vs CPU inference performance"
      ],
      others: [
        "Learn tradeoffs: accuracy vs speed vs power",
        "Read research papers on AI accelerators",
        "Understand hardware/software co-design basics"
      ]
    },
    expert: {
      fundamentals: [
        "NPU/TPU architecture concepts",
        "Advanced memory hierarchy design",
        "Compiler/hardware co-design overview",
        "System-level performance modeling",
        "ASIC design considerations for AI"
      ],
      skills: [
        "Architect full accelerator for target workload",
        "Lead performance tuning and silicon decisions",
        "Collaborate with software/compiler teams",
        "Mentor and guide AI hardware team",
        "Drive product-level optimization"
      ],
      tools: [
        "System-level modeling tools",
        "EDA flows (ASIC path)",
        "Advanced profiling and benchmarking frameworks"
      ],
      projects: [
        "AI accelerator architecture proposal + prototype",
        "Benchmark suite for multiple AI workloads",
        "Memory optimization case study for accelerator",
        "ASIC feasibility study for AI workload",
        "Hardware/software co-design report"
      ],
      others: [
        "Strong demand in AI semiconductor and edge computing",
        "Keep learning latest models and hardware trends",
        "Documentation and clear communication are key"
      ]
    }
  }
},

{
  id: "biomedical-electronics-engineer",
  title: "Biomedical Electronics Engineer",
  domain: "Biomedical",
  level: "Medical Electronics / Bio-Signals",
  salary: "₹4 LPA – ₹25+ LPA",
  companies: "Philips, GE Healthcare, Siemens Healthineers, Medtronic, Abbott",
  shortDescription:
    "Develops electronics for medical devices and bio-signal systems (ECG/EEG/EMG) combining sensors, analog, DSP, and embedded systems.",
  roadmap: {
    basic: {
      fundamentals: [
        "Bio-signals basics: ECG, EEG, EMG",
        "Sensor basics and signal conditioning",
        "Noise and filtering basics",
        "Sampling and ADC basics",
        "Basic electronics for medical devices"
      ],
      skills: [
        "Analog filters and amplification basics",
        "MATLAB signal plotting",
        "Basic embedded prototyping",
        "Simple feature extraction concepts",
        "Basic safety awareness"
      ],
      tools: [
        "MATLAB",
        "Arduino/ESP32",
        "ECG/pulse sensor modules"
      ],
      projects: [
        "ECG signal acquisition + plotting",
        "Pulse monitoring mini-project",
        "Basic noise filtering of ECG signal",
        "Heart rate calculation from sensor",
        "Bio-signal report with plots"
      ],
      others: [
        "Learn safety basics for human-connected devices",
        "Understand signal artifacts and noise sources",
        "Maintain proper documentation"
      ]
    },
    intermediate: {
      fundamentals: [
        "Instrumentation amplifiers basics",
        "DSP: FFT, digital filters",
        "Medical device standards overview (intro)",
        "Signal quality and artifact handling",
        "Basic ML for signal classification (intro)"
      ],
      skills: [
        "Noise reduction techniques",
        "Embedded firmware for sensor acquisition",
        "Feature extraction from signals",
        "Basic classification (optional)",
        "Validation and repeatability checks"
      ],
      tools: [
        "LabVIEW (optional)",
        "STM32/ESP32",
        "Python for signal analysis"
      ],
      projects: [
        "Wearable health monitoring prototype",
        "EEG signal processing and classification demo",
        "EMG signal acquisition and analysis",
        "Real-time filtering on microcontroller",
        "Accuracy validation report"
      ],
      others: [
        "Learn calibration methods",
        "Understand clinical vs prototype difference",
        "Prepare project demo with documentation"
      ]
    },
    expert: {
      fundamentals: [
        "Medical device product lifecycle",
        "Reliability and compliance basics",
        "AI-based medical signal interpretation",
        "Clinical validation concepts",
        "System integration and safety"
      ],
      skills: [
        "Clinical-grade design validation",
        "System-level integration and safety design",
        "Lead product development documentation",
        "Mentor juniors and manage design reviews",
        "Drive regulatory compliance workflow"
      ],
      tools: [
        "Professional medical device toolchain",
        "Validation and testing frameworks",
        "Signal processing + AI pipelines"
      ],
      projects: [
        "End-to-end wearable medical device design",
        "Clinical-grade signal processing pipeline",
        "Medical device compliance documentation project",
        "AI-based diagnostic assistance prototype",
        "Production-ready system validation report"
      ],
      others: [
        "Great domain for research + product engineering",
        "Documentation and compliance are critical",
        "Strong ethics and safety mindset required"
      ]
    }
  }
},
/* =========================================================
   Part-3 Roles — SAME FORMAT (Copy-Paste into roles array)
   ========================================================= */

{
  id: "physical-design-engineer",
  title: "Physical Design Engineer",
  domain: "VLSI",
  level: "Back-End VLSI",
  salary: "₹6 LPA – ₹40+ LPA",
  companies: "Qualcomm, Intel, AMD, NVIDIA, Samsung, Synopsys, Cadence",
  shortDescription:
    "Implements RTL into silicon using floorplanning, placement, CTS, routing, and timing closure (STA) for chip tapeout.",
  roadmap: {
    basic: {
      fundamentals: [
        "CMOS basics and standard cell concepts",
        "RTL → Netlist flow overview",
        "Floorplan basics (core, IO, macros)",
        "Placement and routing basics",
        "Timing basics: setup/hold, slack",
        "Power basics: dynamic/leakage",
        "Clock tree basics"
      ],
      skills: [
        "Understand PD flow stages (FP → Place → CTS → Route)",
        "Read timing reports and identify violations",
        "Basic constraints understanding (SDC overview)",
        "Fix simple timing issues (buffering, sizing)",
        "Basic DRC/LVS awareness"
      ],
      tools: [
        "Synopsys ICC2 / Cadence Innovus (industry)",
        "PrimeTime (STA)",
        "Linux + TCL scripting",
        "VS Code + terminal"
      ],
      projects: [
        "Run basic PD flow on small design (open-source)",
        "Floorplan a small block with macros",
        "Fix simple setup violations using buffering",
        "Create a CTS experiment report",
        "Generate final timing summary report"
      ],
      others: [
        "Learn report reading (timing, congestion, power)",
        "Maintain a checklist for each PD stage",
        "Understand PVT corners at high level"
      ]
    },
    intermediate: {
      fundamentals: [
        "Advanced STA concepts (OCV/AOCV/POCV overview)",
        "IR drop and EM basics",
        "Congestion and routing optimization",
        "Clock skew and latency balancing",
        "Multi-mode multi-corner (MMMC) concepts",
        "Physical synthesis basics"
      ],
      skills: [
        "Timing closure (setup/hold) strategies",
        "Clock tree optimization and skew control",
        "ECO implementation and validation",
        "Fix congestion with floorplan/placement changes",
        "Work with signoff STA reports"
      ],
      tools: [
        "Innovus/ICC2 advanced features",
        "PrimeTime advanced STA",
        "RedHawk (IR/EM - optional)",
        "Joules/PrimePower (power - optional)"
      ],
      projects: [
        "MMMC setup for a block and report results",
        "Fix hold violations using ECO strategies",
        "IR drop analysis mini-study (learning level)",
        "Congestion reduction case study",
        "CTS optimization experiment (skew targets)"
      ],
      others: [
        "Learn PD signoff checklist style documentation",
        "Practice interview questions (timing closure)",
        "Understand ECO flow and best practices"
      ]
    },
    expert: {
      fundamentals: [
        "Full-chip PD methodology and tapeout readiness",
        "Signoff timing closure and correlation",
        "Power integrity signoff strategy",
        "DFM awareness and yield impact",
        "High-performance PPA optimization"
      ],
      skills: [
        "Lead block/chip PD ownership",
        "Drive PPA improvements across iterations",
        "Debug complex timing and SI issues",
        "Coordinate with STA/DFT/Design teams",
        "Mentor team and review PD quality"
      ],
      tools: [
        "Full signoff flow tools (STA + IR/EM + DRC/LVS)",
        "Advanced ECO automation scripts",
        "CI automation for PD regressions"
      ],
      projects: [
        "Tapeout-ready PD closure for a large block",
        "PPA optimization project with measurable gains",
        "Signoff correlation report (implementation vs signoff)",
        "Full-chip integration PD study",
        "PD automation framework for repeated runs"
      ],
      others: [
        "PD engineers must be strong in debugging",
        "Documentation + checklists reduce mistakes",
        "Learn industry signoff expectations deeply"
      ]
    }
  }
},

{
  id: "sta-engineer",
  title: "STA Engineer (Static Timing Analysis)",
  domain: "VLSI",
  level: "Timing Closure",
  salary: "₹7 LPA – ₹45+ LPA",
  companies: "Qualcomm, Intel, AMD, NVIDIA, Samsung, Synopsys, Cadence",
  shortDescription:
    "Performs timing analysis, constraints creation, and timing signoff to ensure chip meets frequency across corners and modes.",
  roadmap: {
    basic: {
      fundamentals: [
        "Setup/hold timing concepts",
        "Clock definitions and uncertainty",
        "Timing paths (reg-to-reg, IO paths)",
        "Basic SDC constraints overview",
        "Standard cells and libraries overview",
        "PVT corners basics"
      ],
      skills: [
        "Read timing reports (WNS/TNS)",
        "Identify critical paths",
        "Basic SDC writing (create_clock, set_input_delay)",
        "Basic timing fixes understanding",
        "Debug false violations (intro)"
      ],
      tools: [
        "Synopsys PrimeTime (industry)",
        "TCL scripting",
        "Linux"
      ],
      projects: [
        "Write SDC for a small design and validate timing",
        "Analyze worst setup paths and summarize",
        "Create timing report automation script",
        "Fix timing by suggesting buffering/pipelining",
        "Corner comparison report (fast/slow)"
      ],
      others: [
        "Learn constraints meaning clearly",
        "Practice timing interview questions",
        "Understand timing reports formatting"
      ]
    },
    intermediate: {
      fundamentals: [
        "MMMC (multi-mode multi-corner)",
        "OCV/AOCV/POCV concepts",
        "Clock gating checks",
        "CDC/RDC impact on timing constraints",
        "Timing exceptions: false path, multicycle"
      ],
      skills: [
        "Build full constraints for SoC/block",
        "Apply correct exceptions and validate them",
        "Analyze hold violations and suggest fixes",
        "Work with PD team for timing closure",
        "Signoff timing closure methodology"
      ],
      tools: [
        "PrimeTime advanced features",
        "PrimeTime SI (optional)",
        "Constraint lint tools (optional)"
      ],
      projects: [
        "MMMC timing setup and signoff-style report",
        "Timing exception validation project",
        "Clock gating checks and fixes report",
        "Hold closure strategy case study",
        "STA automation tool for summary dashboards"
      ],
      others: [
        "Document constraints assumptions carefully",
        "Learn to avoid over-constraining",
        "Develop strong debugging discipline"
      ]
    },
    expert: {
      fundamentals: [
        "Full-chip timing signoff strategy",
        "SI-aware timing closure",
        "Advanced derates and variation modeling",
        "Correlation between tools and signoff flows",
        "Timing closure under aggressive frequency targets"
      ],
      skills: [
        "Lead STA signoff for chip/block",
        "Drive closure with design/PD/DFT teams",
        "Debug complex timing failures",
        "Own timing signoff checklists",
        "Mentor juniors and review constraints quality"
      ],
      tools: [
        "PrimeTime full signoff stack",
        "SI/Noise analysis tools",
        "Automation and regression systems"
      ],
      projects: [
        "Tapeout STA signoff closure report",
        "SI-aware timing optimization case study",
        "Constraints standardization framework",
        "Large SoC timing closure project",
        "STA regression automation pipeline"
      ],
      others: [
        "STA is a high-impact role for tapeout success",
        "Constraints quality = timing quality",
        "Learn how to communicate timing issues clearly"
      ]
    }
  }
},

{
  id: "dft-engineer",
  title: "DFT Engineer (Design for Test)",
  domain: "VLSI",
  level: "DFT / Testability",
  salary: "₹7 LPA – ₹45+ LPA",
  companies: "Intel, Qualcomm, AMD, NVIDIA, Samsung, Synopsys, Siemens EDA",
  shortDescription:
    "Implements scan chains, ATPG, and test structures (MBIST/LBIST) to improve test coverage and manufacturability.",
  roadmap: {
    basic: {
      fundamentals: [
        "Fault models basics (stuck-at)",
        "Scan chain concept",
        "Flip-flops and scan flops difference",
        "Basic ATPG overview",
        "Test coverage basics"
      ],
      skills: [
        "Understand scan insertion flow",
        "Basic DFT constraints awareness",
        "Read DFT reports (coverage, scan chains)",
        "Basic debugging of scan chain issues",
        "Work with RTL and netlists"
      ],
      tools: [
        "Synopsys DFTMAX / Tetramax (industry)",
        "Cadence Modus (industry)",
        "Linux + TCL"
      ],
      projects: [
        "Scan insertion demo on small design",
        "Generate basic ATPG patterns (learning)",
        "Analyze coverage report and improvement ideas",
        "Scan chain debug case study",
        "Write DFT summary report"
      ],
      others: [
        "Learn how DFT impacts area and timing",
        "Understand test modes and reset behavior",
        "Practice DFT interview questions"
      ]
    },
    intermediate: {
      fundamentals: [
        "Transition faults overview",
        "MBIST/LBIST concepts",
        "Compression techniques overview",
        "DFT for low power and clocking",
        "DFT timing and STA interactions"
      ],
      skills: [
        "Implement MBIST controllers (concept level)",
        "Improve coverage and debug untestable logic",
        "DFT-aware constraints and clocking setup",
        "Work with ATE requirements (pattern limits)",
        "Coordinate with PD/STA teams"
      ],
      tools: [
        "Advanced ATPG tools",
        "DFT DRC checkers",
        "Pattern simulation tools"
      ],
      projects: [
        "Transition fault ATPG experiment report",
        "MBIST integration mini-project",
        "Compression impact study on test time",
        "DFT signoff checklist preparation",
        "Pattern simulation and debug"
      ],
      others: [
        "Learn how to handle scan/clock gating safely",
        "Understand test mode pin requirements",
        "Document test architecture clearly"
      ]
    },
    expert: {
      fundamentals: [
        "Full-chip DFT architecture",
        "High coverage signoff and yield impact",
        "Production test flow constraints",
        "Post-silicon failure analysis basics",
        "DFT strategy for complex SoCs"
      ],
      skills: [
        "Lead DFT signoff and closure",
        "Drive coverage improvement initiatives",
        "Optimize test time and cost",
        "Mentor team and review DFT implementation",
        "Work with manufacturing and ATE teams"
      ],
      tools: [
        "Signoff ATPG + diagnosis tools",
        "Production yield dashboards (optional)",
        "Automation for pattern/report generation"
      ],
      projects: [
        "Tapeout-ready DFT signoff project",
        "Full-chip scan/MBIST strategy design",
        "Coverage improvement + RCA documentation",
        "Production failure diagnosis workflow demo",
        "DFT automation scripts for large regressions"
      ],
      others: [
        "DFT is crucial for manufacturability and yield",
        "Communication with ATE and design teams matters",
        "Strong debugging skills are essential"
      ]
    }
  }
},

{
  id: "verification-engineer-uvm",
  title: "Verification Engineer (UVM)",
  domain: "VLSI",
  level: "Digital Verification",
  salary: "₹6 LPA – ₹40+ LPA",
  companies: "Qualcomm, Intel, AMD, NVIDIA, Samsung, Synopsys, Cadence",
  shortDescription:
    "Builds UVM-based verification environments to validate RTL functionality, ensure coverage closure, and catch bugs before tapeout.",
  roadmap: {
    basic: {
      fundamentals: [
        "Digital design basics (FSM, timing)",
        "SystemVerilog basics",
        "Testbench components (driver, monitor)",
        "Randomization basics",
        "Basic assertions (intro)"
      ],
      skills: [
        "Write simple SV testbenches",
        "Generate directed test cases",
        "Basic waveform debug",
        "Understand DUT interfaces",
        "Basic functional coverage concepts"
      ],
      tools: [
        "QuestaSim / VCS / Xcelium",
        "GTKWave / SimVision",
        "VS Code"
      ],
      projects: [
        "Verify UART module with directed tests",
        "Verify FIFO with corner cases",
        "Write simple assertions for protocol checks",
        "Build a small scoreboard for comparison",
        "Create mini regression scripts"
      ],
      others: [
        "Learn bug tracking and reporting",
        "Practice SV/UVM interview questions",
        "Write clear test plans"
      ]
    },
    intermediate: {
      fundamentals: [
        "UVM architecture (agents, env, sequences)",
        "Coverage-driven verification",
        "Scoreboarding and reference models",
        "Protocol verification concepts",
        "Constrained random testing"
      ],
      skills: [
        "Build reusable UVM components",
        "Create coverage plan and closure strategy",
        "Write protocol checkers and assertions",
        "Debug failing random tests",
        "Regression management"
      ],
      tools: [
        "UVM libraries",
        "Python/TCL for automation",
        "Coverage reporting tools"
      ],
      projects: [
        "UVM agent for SPI/I2C protocol",
        "Coverage closure report for IP verification",
        "Scoreboard + reference model integration",
        "Random stress testing for corner cases",
        "Regression dashboard (logs summary)"
      ],
      others: [
        "Learn to reduce simulation time smartly",
        "Maintain verification plan document",
        "Understand how to triage failures quickly"
      ]
    },
    expert: {
      fundamentals: [
        "SoC-level verification strategy",
        "Formal verification basics",
        "Performance verification (throughput/latency)",
        "Emulation/FPGA prototyping overview",
        "Signoff methodology and quality metrics"
      ],
      skills: [
        "Lead DV signoff and coverage closure",
        "Own verification architecture for complex blocks",
        "Develop reusable VIP",
        "Mentor juniors and improve verification quality",
        "Drive bug prevention using assertions/formal"
      ],
      tools: [
        "Formal tools (JasperGold - optional)",
        "Emulation tools (optional)",
        "CI/CD for regressions"
      ],
      projects: [
        "SoC DV environment with reusable VIP",
        "Formal + simulation hybrid verification demo",
        "Performance stress test suite",
        "Signoff verification plan + closure report",
        "Reusable verification framework template"
      ],
      others: [
        "Verification is high-demand and core to tapeout",
        "Focus on coverage + bug prevention",
        "Communication with design team is critical"
      ]
    }
  }
},

{
  id: "embedded-systems-engineer",
  title: "Embedded Systems Engineer",
  domain: "Embedded",
  level: "Firmware / Embedded Software",
  salary: "₹3 LPA – ₹25+ LPA",
  companies: "Bosch, Siemens, Samsung, Qualcomm, Honeywell, Tata Elxsi",
  shortDescription:
    "Develops firmware for microcontrollers and embedded systems using C/C++ with peripherals, RTOS, and hardware interfacing.",
  roadmap: {
    basic: {
      fundamentals: [
        "C programming basics",
        "Microcontroller basics (GPIO, timers)",
        "UART/I2C/SPI basics",
        "Interrupts and PWM basics",
        "ADC basics",
        "Basic electronics and sensors"
      ],
      skills: [
        "Write embedded C drivers",
        "Read datasheets and reference manuals",
        "Debug using serial logs",
        "Interface sensors and actuators",
        "Basic PCB awareness"
      ],
      tools: [
        "Arduino IDE / PlatformIO",
        "STM32CubeIDE (optional)",
        "Logic Analyzer (optional)",
        "Serial monitor tools"
      ],
      projects: [
        "Sensor reading + display (I2C OLED)",
        "PWM motor control project",
        "UART communication logger",
        "Temperature monitoring with alert",
        "Basic data acquisition system"
      ],
      others: [
        "Learn proper coding style and modular design",
        "Use GitHub for embedded projects",
        "Document wiring diagrams clearly"
      ]
    },
    intermediate: {
      fundamentals: [
        "RTOS concepts (tasks, semaphores)",
        "DMA basics",
        "Low power modes in MCU",
        "Bootloader basics",
        "Embedded debugging methods"
      ],
      skills: [
        "Write RTOS-based firmware",
        "Design driver layers (HAL/LL)",
        "Optimize memory and performance",
        "Implement OTA updates (IoT)",
        "Unit testing basics for firmware"
      ],
      tools: [
        "FreeRTOS",
        "JTAG/SWD debug tools",
        "Wireshark (for IoT debugging)"
      ],
      projects: [
        "RTOS-based sensor fusion system",
        "Bootloader + firmware update demo",
        "Low power wearable device prototype",
        "IoT MQTT device with secure communication",
        "DMA-based high-speed data capture"
      ],
      others: [
        "Learn debugging with breakpoints and trace",
        "Write proper test cases",
        "Understand power/performance tradeoffs"
      ]
    },
    expert: {
      fundamentals: [
        "Embedded system architecture design",
        "Real-time scheduling and performance tuning",
        "Security for embedded devices",
        "Reliability and fault tolerance",
        "Production-level firmware lifecycle"
      ],
      skills: [
        "Own firmware architecture for products",
        "Lead integration with hardware/software teams",
        "Drive reliability improvements",
        "Mentor juniors and enforce best practices",
        "Handle complex bug RCA"
      ],
      tools: [
        "CI/CD for firmware builds",
        "Advanced debugging tools",
        "Static analysis tools (optional)"
      ],
      projects: [
        "Production-grade firmware framework",
        "Embedded security hardening project",
        "Fault-tolerant system design case study",
        "End-to-end embedded product prototype",
        "Firmware release + documentation pipeline"
      ],
      others: [
        "Embedded engineers must be strong in debugging",
        "Documentation and testing are crucial",
        "Keep learning new MCUs and protocols"
      ]
    }
  }
},

{
  id: "robotics-engineer",
  title: "Robotics Engineer",
  domain: "Robotics",
  level: "Robotics / Automation",
  salary: "₹4 LPA – ₹35+ LPA",
  companies: "ABB, KUKA, Boston Dynamics, Tesla, NVIDIA, Tata Advanced Systems",
  shortDescription:
    "Builds robots using sensors, perception, control systems, and ROS for navigation, manipulation, and automation.",
  roadmap: {
    basic: {
      fundamentals: [
        "Linear algebra basics (vectors, matrices)",
        "Kinematics basics (forward/inverse)",
        "Sensors: IMU, LiDAR, camera basics",
        "Control basics (PID)",
        "ROS basics (nodes, topics, services)"
      ],
      skills: [
        "Python/C++ basics for robotics",
        "Basic ROS publisher/subscriber",
        "Read sensor data and visualize",
        "Basic simulation usage",
        "Debugging using RViz plots"
      ],
      tools: [
        "ROS/ROS2",
        "Gazebo",
        "RViz",
        "OpenCV basics"
      ],
      projects: [
        "Line follower robot",
        "Obstacle avoidance robot using ultrasonic/LiDAR",
        "ROS turtlebot simulation navigation",
        "PID motor speed controller",
        "Camera-based object detection demo"
      ],
      others: [
        "Maintain GitHub for robotics projects",
        "Learn sensor calibration basics",
        "Document experiments with videos/screenshots"
      ]
    },
    intermediate: {
      fundamentals: [
        "SLAM basics (mapping + localization)",
        "Path planning algorithms (A*, DWA)",
        "Sensor fusion basics (Kalman filter)",
        "Robot dynamics basics",
        "ROS navigation stack overview"
      ],
      skills: [
        "Implement SLAM in simulation/real robot",
        "Tune navigation parameters",
        "Integrate LiDAR + IMU + encoder data",
        "Build autonomous navigation pipeline",
        "Optimize robot performance"
      ],
      tools: [
        "Cartographer / GMapping (ROS)",
        "RQT tools",
        "Python data analysis"
      ],
      projects: [
        "2D LiDAR SLAM mapping project",
        "Autonomous navigation in indoor map",
        "Sensor fusion for pose estimation",
        "Robot path planning and obstacle avoidance",
        "ROS package creation and documentation"
      ],
      others: [
        "Learn how to debug TF frames issues",
        "Understand real-world noise and drift",
        "Practice robotics interview questions"
      ]
    },
    expert: {
      fundamentals: [
        "Advanced SLAM and loop closure",
        "Multi-sensor fusion and optimization",
        "Robotics system architecture design",
        "Real-time constraints and optimization",
        "Safety and reliability in robotics"
      ],
      skills: [
        "Lead robotics product development",
        "Build robust SLAM + navigation for production",
        "Optimize compute and real-time performance",
        "Mentor team and drive architecture decisions",
        "Deploy robots in real environments"
      ],
      tools: [
        "Ceres Solver (optimization)",
        "Advanced ROS2 deployment workflows",
        "CI pipelines for robotics"
      ],
      projects: [
        "Production-level navigation stack deployment",
        "Multi-sensor SLAM with loop closure",
        "Robotics system performance optimization case study",
        "Real robot deployment and field debugging report",
        "Robotics architecture documentation project"
      ],
      others: [
        "Robotics needs strong math + debugging skills",
        "Real-world testing is critical",
        "Documentation helps team scale faster"
      ]
    }
  }
},

{
  id: "fpga-design-engineer",
  title: "FPGA Design Engineer",
  domain: "VLSI",
  level: "FPGA / RTL",
  salary: "₹4 LPA – ₹30+ LPA",
  companies: "Xilinx (AMD), Intel (Altera), Qualcomm, Samsung, ISRO, DRDO",
  shortDescription:
    "Designs digital hardware on FPGA using Verilog/SystemVerilog, simulation, timing closure, and hardware debugging.",
  roadmap: {
    basic: {
      fundamentals: [
        "Digital logic basics",
        "Verilog/SystemVerilog basics",
        "Clocking and reset basics",
        "Basic timing concepts",
        "FPGA architecture overview (LUT, FF, BRAM)"
      ],
      skills: [
        "Write synthesizable RTL",
        "Simulate RTL and debug waveforms",
        "Implement design on FPGA board",
        "Use constraints file basics (XDC)",
        "Basic hardware debugging"
      ],
      tools: [
        "Xilinx Vivado / Intel Quartus",
        "ModelSim/QuestaSim",
        "ILA (Integrated Logic Analyzer)",
        "GTKWave"
      ],
      projects: [
        "UART TX/RX on FPGA",
        "PWM motor controller on FPGA",
        "SPI controller with sensor interface",
        "BRAM-based FIFO design",
        "Simple RISC-V core (learning)"
      ],
      others: [
        "Start using ILA for debugging early",
        "Maintain GitHub for FPGA projects",
        "Write clean documentation for constraints"
      ]
    },
    intermediate: {
      fundamentals: [
        "Pipelining and throughput optimization",
        "Clock domain crossing basics",
        "AXI streaming basics",
        "DDR basics overview",
        "FPGA resource and timing analysis"
      ],
      skills: [
        "Timing closure strategies",
        "Design multi-clock systems safely",
        "Use AXI IP integration",
        "Optimize for performance and area",
        "Write reusable FPGA IP blocks"
      ],
      tools: [
        "Vivado IP Integrator",
        "Timing analyzer reports",
        "ILA advanced triggers"
      ],
      projects: [
        "AXI-stream data processing pipeline",
        "Async FIFO for CDC",
        "Image processing pipeline on FPGA",
        "High-speed ADC capture and streaming",
        "FPGA-based accelerator demo"
      ],
      others: [
        "Learn to read timing reports deeply",
        "Understand constraints and false paths",
        "Practice FPGA interview questions"
      ]
    },
    expert: {
      fundamentals: [
        "High-speed interfaces (PCIe/SerDes) overview",
        "FPGA system architecture design",
        "Power and thermal considerations",
        "Hardware/software co-design",
        "Production deployment considerations"
      ],
      skills: [
        "Lead FPGA architecture and integration",
        "Debug complex board-level issues",
        "Optimize performance at system level",
        "Mentor juniors and review RTL quality",
        "Deliver stable FPGA releases"
      ],
      tools: [
        "Advanced FPGA debug tools",
        "CI for FPGA builds (optional)",
        "Board bring-up workflows"
      ],
      projects: [
        "FPGA product-level accelerator system",
        "High-speed data acquisition platform",
        "PCIe streaming design (learning)",
        "Full system validation and performance report",
        "FPGA release pipeline documentation"
      ],
      others: [
        "FPGA engineers must be strong in debugging",
        "Documentation + constraints discipline is key",
        "Keep improving timing closure expertise"
      ]
    }
  }
},

{
  id: "power-electronics-engineer",
  title: "Power Electronics Engineer",
  domain: "Power",
  level: "Power Systems / Electronics",
  salary: "₹4 LPA – ₹25+ LPA",
  companies: "Tesla, ABB, Siemens, Schneider Electric, Tata Power, L&T",
  shortDescription:
    "Designs and tests converters/inverters, motor drives, and power systems for EVs, renewable energy, and industrial applications.",
  roadmap: {
    basic: {
      fundamentals: [
        "Semiconductor devices: diode, MOSFET, IGBT basics",
        "Basic converter types (buck/boost)",
        "Inductor/capacitor selection basics",
        "PWM basics",
        "Basic motor fundamentals"
      ],
      skills: [
        "Design basic DC-DC converter calculations",
        "Simulate power circuits",
        "Measure voltage/current safely",
        "Basic PCB power layout awareness",
        "Write test reports"
      ],
      tools: [
        "MATLAB/Simulink",
        "LTspice / PSIM",
        "Oscilloscope + power probes"
      ],
      projects: [
        "Buck converter simulation + design report",
        "Boost converter design for LED driver",
        "PWM motor speed controller demo",
        "Battery charging concept project",
        "Efficiency calculation and measurement report"
      ],
      others: [
        "Learn safety for high voltage systems",
        "Understand thermal management basics",
        "Practice component selection from datasheets"
      ]
    },
    intermediate: {
      fundamentals: [
        "Inverters and motor drives basics",
        "Control methods (PI control)",
        "EMI/EMC basics for power circuits",
        "Thermal design and heat sinks",
        "Efficiency optimization"
      ],
      skills: [
        "Design inverter and gate driver basics",
        "Closed-loop control implementation",
        "EMI noise reduction techniques",
        "Hardware debugging and failure analysis",
        "Design for reliability"
      ],
      tools: [
        "Simulink control models",
        "PSIM advanced simulations",
        "Thermal analysis tools (optional)"
      ],
      projects: [
        "Inverter simulation for BLDC motor",
        "Closed-loop buck converter control demo",
        "EMI reduction case study",
        "Gate driver and switching loss analysis",
        "Thermal analysis mini-project"
      ],
      others: [
        "Learn PCB layout rules for switching power",
        "Document design tradeoffs clearly",
        "Understand protection circuits (OCP/OVP)"
      ]
    },
    expert: {
      fundamentals: [
        "EV powertrain architecture overview",
        "Advanced control strategies",
        "Wide bandgap devices (SiC/GaN) basics",
        "Reliability and lifetime estimation",
        "Production validation and compliance"
      ],
      skills: [
        "Lead power converter product development",
        "Optimize power density and efficiency",
        "Drive compliance testing strategy",
        "Mentor juniors and review designs",
        "Handle field failure RCA"
      ],
      tools: [
        "Advanced simulation + validation workflows",
        "Power analyzer tools",
        "Production testing setups"
      ],
      projects: [
        "EV inverter design study + validation plan",
        "High-efficiency converter optimization project",
        "GaN-based converter prototype study",
        "Production test and reliability documentation",
        "Field issue RCA report"
      ],
      others: [
        "Power electronics needs strong safety mindset",
        "Hardware testing is essential",
        "Keep learning latest devices and standards"
      ]
    }
  }
},

{
  id: "iot-engineer",
  title: "IoT Engineer",
  domain: "IoT",
  level: "Embedded + Cloud",
  salary: "₹4 LPA – ₹28+ LPA",
  companies: "Bosch, Siemens, Honeywell, Amazon, Google, Tata Elxsi, Wipro",
  shortDescription:
    "Builds IoT devices and systems by connecting sensors and embedded firmware to cloud platforms with dashboards and analytics.",
  roadmap: {
    basic: {
      fundamentals: [
        "Sensors and microcontrollers basics",
        "WiFi/Bluetooth basics",
        "MQTT/HTTP basics",
        "Basic networking concepts",
        "Data formats (JSON)"
      ],
      skills: [
        "ESP32/Arduino programming",
        "Publish sensor data to cloud",
        "Basic REST API usage",
        "Build simple dashboards",
        "Basic debugging and logging"
      ],
      tools: [
        "ESP32 / Arduino",
        "MQTT broker (Mosquitto)",
        "Node-RED (optional)",
        "Firebase (optional)"
      ],
      projects: [
        "Temperature monitoring IoT device",
        "MQTT-based sensor data pipeline",
        "Smart home light control",
        "IoT dashboard for live data",
        "Device alert system (email/notification)"
      ],
      others: [
        "Learn security basics (passwords, TLS intro)",
        "Document wiring and code properly",
        "Use GitHub for IoT projects"
      ]
    },
    intermediate: {
      fundamentals: [
        "OTA updates basics",
        "Device provisioning concepts",
        "Cloud IoT platforms overview",
        "Database basics (time-series data)",
        "Security basics (TLS, certificates)"
      ],
      skills: [
        "Implement OTA update pipeline",
        "Secure MQTT communication",
        "Build scalable device backend (optional)",
        "Analyze IoT data trends",
        "Device reliability improvements"
      ],
      tools: [
        "AWS IoT / Azure IoT (optional)",
        "InfluxDB/Grafana (optional)",
        "Python data processing"
      ],
      projects: [
        "Secure IoT device with TLS MQTT",
        "OTA firmware update demo",
        "IoT fleet monitoring dashboard",
        "Edge analytics mini-project",
        "IoT security assessment report"
      ],
      others: [
        "Learn device lifecycle management",
        "Practice real-world reliability testing",
        "Write clear system architecture docs"
      ]
    },
    expert: {
      fundamentals: [
        "Large-scale IoT architecture design",
        "Security and compliance for IoT",
        "Edge AI concepts",
        "Production monitoring and maintenance",
        "System scalability planning"
      ],
      skills: [
        "Lead IoT product architecture end-to-end",
        "Design secure provisioning and key management",
        "Build monitoring and alerting systems",
        "Mentor juniors and manage deployments",
        "Handle field failures and RCA"
      ],
      tools: [
        "Cloud monitoring tools",
        "CI/CD pipelines for firmware + cloud",
        "Security scanning tools"
      ],
      projects: [
        "Production-ready IoT platform design",
        "IoT security hardening and audit project",
        "Edge AI IoT device prototype",
        "Fleet monitoring and analytics pipeline",
        "Field issue RCA + improvements plan"
      ],
      others: [
        "IoT requires strong security and reliability mindset",
        "Documentation is critical for scaling systems",
        "Keep learning new protocols and platforms"
      ]
    }
  }
}

];
