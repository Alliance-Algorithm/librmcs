# LibRMCS

librmcs is a highly streamlined version of the RoboMaster Control System (RMCS) without a lower-level controller, available at [RMCS GitHub Repository](https://github.com/Alliance-Algorithm/RMCS).

## Why Choose LibRMCS?

Compared to traditional lower-level controller systems:

- Lower difficulty: Leveraging strong encapsulation, librmcs significantly reduces the development difficulty for users. Developers are spared from embedded-related knowledge such as interrupts, registers, and clock trees, and do not need to learn HAL and STM32CubeMX. Basic programming skills suffice to quickly develop various control programs.

  This capability also enables non-electronics team members (such as mechanical/vision) to write simple test control code. Users can develop and debug control code on any non-real-time operating system (Windows/Linux) using any programming language (C++/Python). Even with minimal programming language foundation, usable control code can be rapidly developed, aided by modern AI tools.

- Access to more resources: Running on desktop computers, librmcs can harness the computing power and storage capacity of desktop systems. For instance, when tuning parameters for mechanisms (like gimbals), librmcs facilitates writing test control code to gather information directly into CSV files on the hard drive, easing further analysis using tools like Matlab.

- Better language ecosystem: As a lightweight lower-level control library, librmcs seamlessly integrates with ecosystems of various programming languages. For example, when testing newly fabricated mechanical structures, librmcs can be used to develop test control code, with results transmitted in real-time via Python's requests library to collaboration tools like Feishu for information gathering and sharing.

Compared to the full RoboMaster Control System (RMCS):

- Easier learning curve: RMCS, designed for comprehensive control across all units, entails a larger codebase and introduces complex concepts (components, subcomponents, plugins, dependency analysis, update sequences), posing significant cognitive load for beginners. 

  librmcs, on the other hand, simplifies the learning process by removing these complexities, exposing concise core interfaces to users, offering greater flexibility, and smoothing the learning curve.

- Extremely lightweight and cross-platform: For convenience in development, RMCS heavily depends on tools like ROS2, OpenCV, and Eigen, and uses Docker containers for both development and deployment environments. This restricts RMCS to specific operating systems and languages, mandating development on Linux using C++.

  librmcs addresses these limitations by removing all removable dependencies, making it more suitable than RMCS for rapid development in small-scale projects such as testing and data collection.
  