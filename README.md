Semplice struttura del progetto (simulazione di `tree`):

.
├── README.md
├── data
│   ├── 01_current_model
│   │   ├── RL_calibration
│   │   │   ├── 12Vtimeval.fig
│   │   │   ├── 12Vtimeval.mat
│   │   │   ├── 15Vtimeval.fig
│   │   │   ├── 15Vtimeval.mat
│   │   │   └── ...
│   │   ├── RL_Circuit_Calibration_old
+│   │   │   └── ...
│   │   ├── time
│   │   └── ...
│   ├── 02_current_control
│   │   ├── current_validation_old
│   │   ├── frequency
│   │   └── time
│   └── 03_position
│       ├── FB
│       ├── LQR
│       └── PP
├── Experiments
│   └── Ramp Voltage Input
├── doc
│   └── phases_slides.txt
├── img
│   ├── current_control
│   │   ├── frequency
│   │   └── time
│   ├── FB_validation
+│   │   ├── frequency
│   │   └── time
│   ├── LQRI_validation
│   │   ├── frequency
│   │   └── simulation
│   └── models
├── matlab
│   ├── MagneticLevitation_Template.slx
│   ├── Model.m
│   ├── 01_FB_current
│   │   ├── PI_requirement.m
│   │   └── RL_model.m
│   ├── 02_FB_position
│   │   ├── FB_Position_Control.m
│   │   └── Simulink_FB_Position_Control.slx
│   ├── 03_PP_position
│   │   ├── plot_PPI_simulation_data.m
│   │   └── PP_NL_simulation.slx
│   └── NL_model
│       ├── NL_function_block.slx
│       └── NL_model.m
├── paper
├── report
│   ├── report.tex
│   └── sections
│       ├── 01_introduction
│       ├── 02_system_model
│       └── 03_open_loop_analysis
└── README.md (questo file)

Nota: le directory sono mostrate in forma compatta; alcuni file sono omessi con "...".
