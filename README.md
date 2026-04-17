

# 🤖 **Ashish Paka – Robotics & Autonomous Systems Engineer**

[![Portfolio](https://img.shields.io/badge/Portfolio-ashish--paka.netlify.app-brightgreen?style=flat&logo=netlify&logoColor=white)](https://ashish-paka.netlify.app/)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-ashish--autonomousrobotics-blue?style=flat&logo=linkedin)](https://linkedin.com/in/ashish-autonomousrobotics)
[![GitHub](https://img.shields.io/badge/GitHub-Ashish--Paka-black?style=flat&logo=github)](https://github.com/Ashish-Paka)
📧 [ashishpaka1998@gmail.com](mailto:ashishpaka1998@gmail.com)

---

## 🔥 About Me

I am a passionate **Robotics and AI Engineer** with deep expertise in **advanced perception, multi-robot coordination, reinforcement learning, and control systems**. My academic journey—earning an **M.S. in Robotics & Autonomous Systems** from ASU and a **B.Tech in Mechanical Engineering** from Manipal Institute of Technology—has provided me with a solid foundation in mathematics, control theory, kinematics, and dynamics. These skills are crucial for designing and deploying real-world autonomous systems.

I have refined my abilities through diverse hands-on projects and research, including:
- **Robotics Projects and research during my M.Sc. Robotics and Autonomous Systems at ASU**
- **Research at LOGOS Robotics Lab: ( 2025 Jan onwards )**  
  Under Prof. Nakul Gopalan, I contributed to a *shared action space for human–robot collaboration* by combining skill learning, computer vision, and reinforcement learning.



I thrive on integrating robotics software with cutting-edge mechanical design to create transformative solutions that push the boundaries of autonomous systems.

---

## 🧠 Research Interests & Technical Focus

I concentrate on the following core areas—blending fundamental principles with advanced research methodologies:

- **Machine Learning, Deep Learning, Generative AI and Transformers and Reinforcement Learning:**
  *Developing new architectures in these new cutting edge fields based on linear algebra and data science. Expeienced in multi-modal gen AI development.*
  
- **Swarm Robotics & Multi-Agent Systems:**  
  *Exploring collaborative mapping, distributed decision-making, and reinforcement learning (MARL) to enable robust autonomous exploration and coordination.*

- **Advanced Perception & Computer Vision:**  
  *Leveraging deep learning, transformer-based models, and sensor fusion (LiDAR, TOF cameras, and conventional cameras) for real-time environmental understanding and occlusion-aware mapping.*

- **Humanoid Robotics & Shared Action Space Learning:**  
  *Developing adaptive models that empower humanoid systems to intuitively learn from and interact with humans using skill learning, action recognition, and self-supervised control.*

- **Autonomous Driving (AD):**  
  *Employing semantic scene completion, BEV mapping, and transformer-based perception to support occlusion-aware navigation and real-time decision-making in dynamic environments.*

- **Control Systems & Kinematics:**  
  *Utilizing classical PID, advanced motion planning, inverse kinematics, and Jacobian-based corrections to achieve precision in robotic motion and industrial automation.*

- **Mathematics & Computational Methods:**  
  *Grounded in linear algebra, dynamics, and advanced system modeling to develop robust algorithms for perception, sensor fusion, and optimization.*



## 🔥 Bachelors Research and Industry Work Experience

- **Mechanical Design & Simulation:**  
  *Applying advanced CAD tools (CREO, SOLIDWORKS, CATIA, Fusion360, Inventor) and simulation platforms (ANSYS for FEA/CFD) to design and analyze complex mechanical structures.*
  
- **Industrial & Aerospace Projects:**  
  From designing rocket structures (*Project Arya* and *Project Vyom*) at India's top student rocketry team "thrustMIT" as a rocket engineer. Working for LTTS as a Mechanical Engineer. Developing plug gauge tools at BHEL and simulating Total Knee Replacement Implants, I have applied engineering principles across varied domains.

- **Innovation & Patents:**  
  Recognized for innovation as the patent holder of “A SYSTEM FROM EJECTION” (Patent No: 506725).


---

## 🔬 Research & Projects

### 1️⃣ PerSPARAMA — Perception of a Self-worth Perspective Agent with Reality-Aligned Mission Autonomy
*LOGOS Robotics Lab, Arizona State University · Jan 2025 – Present*

- **Overview:**
  A unified cognitive architecture for solo autonomous agents operating in GPS-denied, communication-degraded environments — a robot that knows it is tired, learns from what happens to it, and cares enough about itself to survive.
- **Technical Details:**
  - **Perception ↔ Self coupling:** Sparse voxel transformer world model and a homeostatic self-model (`H(t) ∈ [0,1]⁶`) coupled via bidirectional cross-attention into a joint situational latent `Z_relation`.
  - **Self-worth as utility, not constraint:** Action selection by an action-chunked transformer governed by **dual-utility PPO** with a learned, state-conditioned exchange rate `α(t)` arbitrating mission reward versus self-preservation reward.
  - **Reality alignment:** A reality-alignment loss `L_reality` jointly updates world and self models from physical outcomes — the agent grows through experience rather than only operating.
  - **Generalized skill boundary detection:** Three-way classification (execute / adapt / improvise) applied to *every* decision — path, sensor allocation, return timing — extending ACT-LoRA beyond named-skill invocation.
  - **Embodiment-agnostic** via a formal interface contract: identical cognitive stack across wheeled, aerial, aquatic, and humanoid platforms.
- **Repository:**
  [PerSPARAMA](https://github.com/Ashish-Paka/PerSPARAMA-Perception-of-a-Self-worth-Perspective-Agent-with-Reality-Aligned-Mission-Autonomy)
  _🧠🤖 An agent that perceives the world through who it currently is._

---

### 2️⃣ A.L.B.E.R.C. — Autonomous Learning Bot to Explore, React and Collaborate
*Independent research robot · Jan 2026 – Present*

- **Overview:**
  A sub-$1000, ~50 W three-tier differential-drive indoor mobile robot for autonomous 3D mapping, dynamic obstacle evasion, and natural-language human-robot collaboration.
- **Hardware Stack:**
  Jetson Orin Nano Super (67 TOPS) + Arduino Mega 2560 + docked smartphone; Unitree L1 4D LiDAR (360×90° FOV), Stereolabs ZED Mini (visual-inertial odometry), ICM-20948 IMU, ultrasonic proximity, quadrature encoders.
- **Technical Details:**
  - **Sensor Fusion & SLAM:** Extended Kalman Filter across LiDAR + stereo VIO + IMU + wheel encoders; dual 2D/3D SLAM with **SLAM Toolbox + RTAB-Map** and persistent cross-session maps.
  - **Planning:** Three runtime-selectable global planners (**A\***, **RRT\***, **Voronoi GVD**) with **MPPI** local trajectory optimization at 20 Hz.
  - **LLM Cognition:** Provider-agnostic LLM interface (OpenAI / Anthropic Claude / Google Gemini) for voice-driven task decomposition, with Gemini VLM grounding the compound sensor-fusion view.
  - **Mission-context-dependent dynamic reactivity:** 20 Hz time-to-collision evasive control as primary control authority, gated by LLM-designated interaction targets — the robot evades everything it is not actively collaborating with.
  - **Distributed ROS2 compute:** Any node launchable on the Jetson or an offload laptop; full Docker (ARM64 + x86) deployment.
- **Repository:**
  [A.L.B.E.R.C.](https://github.com/Ashish-Paka/A.L.B.E.R.C.---Autonomous-Learning-Bot-to-Explore-React-and-Collaborate)
  _🛰️🤖 A pocket-budget research platform for embodied perception, planning, and HRI._

---

### 3️⃣ Shared Action Space for Human–Robot Collaboration
- **Overview:**
  Developing a framework that enables humanoid robots to predict and adapt to human actions by learning a shared action space.
- **Technical Details:**
  - **Perception & Learning:** Utilizes transformer-based vision models and deep reinforcement learning for self-supervised skill acquisition.
  - **Control & Adaptation:** Integrates probabilistic inference to allow real-time adjustments in collaborative tasks.
- **Impact:**
  Enhances adaptive human–robot interaction, paving the way for robots that generalize actions and can request demonstrations when needed. This line of work matured into [PerSPARAMA](https://github.com/Ashish-Paka/PerSPARAMA-Perception-of-a-Self-worth-Perspective-Agent-with-Reality-Aligned-Mission-Autonomy) and the cross-embodiment skill work [VLM_Video_description](https://github.com/Ashish-Paka/VLM_Video_description).

---

### 4️⃣ Swarm Robotics for Collaborative Autonomous Mapping
*Aug 2024 – Dec 2024 · Southwest Robotics Symposium 2024 paper presentation*

- **Overview:**
  A distributed multi-robot system for real-time exploration, mapping, and data fusion — pioneering Voronoi-pattern swarm exploration for optimal area coverage.
- **Technical Details:**
  - **Sensor Integration:** TOF cameras + LiDAR for environmental data capture, attaining **5× faster 3D reconstruction** vs. sequential approaches.
  - **SLAM & Coordination:** ROS2 with `gmapping` and Hector SLAM, tested in Gazebo with TurtleBot3; graph-based pose-graph optimization (G2O) with loop closure for globally consistent maps.
  - **Reinforcement Learning:** Gymnasium + Deep Q-Network for intelligent exploration under uncertainty.
  - **Data Fusion:** Bayesian occupancy-grid fusion to merge individual robot maps coherently.
- **Repository:**
  [Swarm Mapping](https://github.com/Ashish-Paka/Multi-Robot_Systems_CollaborativeMapping)
  _🤖✨ Robots working in unison make our world smarter!_

---

### 5️⃣ Optimized VoxFormer for Autonomous Driving
*Jan 2024 – May 2024*

- **Overview:**
  A 75% resource-efficient 3D Semantic Scene Completion Voxformer model that reconstructs occluded scenes from camera-only inputs — a critical capability for autonomous driving perception.
- **Technical Details:**
  - **Training:** SemanticKITTI (3.6 billion labelled points) with mixed-precision PyTorch distributed training; **+10% IoU** improvement on occluded voxel estimation.
  - **Architecture:** Sparse voxel transformer (ViT) + occupancy grid mapping + Bird's Eye View (BEV) projections for pedestrian/vehicle detection.
  - **Deployment & Tooling:** CARLA simulator integration for real-time inference at **20 FPS**, TensorBoard for loss visualization, Open3D for 3D voxel rendering, Weights & Biases for hyperparameter optimization tracking; targeted for NVIDIA Jetson AGX Xavier.
- **Repository:**
  [VoxFormer](https://github.com/Ashish-Paka/Voxformer_AutonomousDriving_TransformerModel)
  _🚗💨 Enhancing safety through smarter scene understanding!_

---

### 6️⃣ Expressive Robot Hand – AI-driven Human–Robot Interaction
- **Overview:**
  Created a robotic hand that mimics human gestures through advanced computer vision and adaptive control.
- **Technical Details:**
  - **Vision Systems:** Utilizes MediaPipe and OpenPose for precise hand tracking.
  - **Sequence Prediction:** Employs GRU-based neural networks for accurate gesture sequence prediction.
  - **Reinforcement Learning:** Implements policy gradient methods (e.g., PPO) for dynamic optimization of control strategies.
- **Repository:**
  [Expressive Robot Hand](https://github.com/Ashish-Paka/Expressive-Robot-Hand)
  _✋🤖 Bringing lifelike interaction to robotics!_

---

### 7️⃣ 6-DOF Trajectory Optimization for Industrial Robotic Arms
- **Overview:**
  Developed advanced path planning algorithms to achieve precise, collision-free motion in 6-degree-of-freedom industrial robots.
- **Technical Details:**
  - **Motion Planning:** Integrates RRT*, CHOMP, and STOMP algorithms for effective trajectory planning.
  - **Control & Kinematics:** Utilizes Jacobian-based corrections and PID control loops to minimize end-effector error and optimize movement.
- **Repository:**
  [6DOF Cobot Path Planning](https://github.com/Ashish-Paka/6DOFCOBOT_TrajectoryPlanning_Kinematics)
  _🔄⚙️ Precision in every move!_

---

### 8️⃣ AR-VR Pass-through and Virtual Reality Environment
- **Overview:**
  Developed immersive AR/VR applications for interactive simulation and gaming, blending cutting-edge software and hardware integration.
- **Technical Details:**
  - **Development Tools:** Uses Meta Quest Development Hub and Unity for immersive AR/VR experiences.
  - **User Interaction:** Designs intuitive interfaces that bridge real and virtual worlds, enhancing user engagement.
- **Repository:**
  Explore my [AR/VR Projects](https://github.com/stars/Ashish-Paka/lists/ar-vr-development)

---

### 9️⃣ Image Recognition-based ML-DL Algorithms Comparison for E-commerce
- **Overview:**
  Designed ML/DL models for classifying and sorting e-commerce product images, improving personalized recommendation systems.
- **Technical Details:**
  - **Data Processing:** Leverages deep learning frameworks for image recognition and classification.
  - **Personalization:** Enhances user experience by tailoring recommendations based on automated product sorting.
- **Repository:**
  [MLDL Image Classification Methods Comparison](https://github.com/Ashish-Paka/ML-DL-Models-Comparison_Images_FashionMNISTDataset-Classification_SearchPersonalisation.git)

---

## 🛠 Technical Skills

### Robotics Software & Simulation
- **Middleware & Frameworks:**  
  ROS 1/2 (Catkin, Nav2, MoveIt), V-Rep, Gazebo, PyBullet, MuJoCo.
- **AR/VR Development:**  
  Meta Quest Development Hub and Unity AR/VR.

### Advanced Perception & Computer Vision
- **Libraries & Tools:**  
  OpenCV, TensorFlow, PyTorch.
- **Techniques:**  
  Transformer-based architectures (ViTs), sensor fusion (integrating LiDAR, TOF cameras, and visual sensors), and BEV mapping.

### Control, Kinematics & Motion Planning
- **Control Systems:**  
  Classical PID, inverse kinematics, and modern control theories.
- **Motion Planning Algorithms:**  
  RRT*, CHOMP, STOMP, A*.
- **Mathematics & Modelling:**  
  Linear algebra, dynamics, and differential equations for robust system modeling.

### Mechanical Design & Simulation
- **CAD & Simulation Software:**  
  CREO, SOLIDWORKS, CATIA, Autodesk Fusion360, Inventor; ANSYS (FEA, CFD, transient thermal analysis).
- **Manufacturing & Design:**  
  DFMA/DFMAE, rapid prototyping (3D printing), and structural design for aerospace and rocket systems.

### Programming & Embedded Systems
- **Languages:**  
  Python, C/C++, Java, Embedded C, CUDA, MATLAB, Arduino, Bash, LaTeX.
- **Development Tools:**  
  Git, Docker, IDEs across Linux, Windows, Android, and iOS.

### Additional Engineering Domains
- **Mechanical Engineering:**  
  Expertise in Turbo Machines, Fluid Mechanics, Heat Transfer, Strength of Materials, and FEA.
- **Control Theory & Mathematics:**  
  Courses in Linear Algebra, Modelling and Control of Robots, Dynamics and Control, and Advanced System Modelling.

---

## 💼 Professional Experience

### Engineer – Larsen & Toubro Technology Services (LTTS), Bangalore, India  ⚙️⚙️
*September 2020 – June 2023*  
- **Role:** Mechanical product design and analysis engineer.
- **Responsibilities:**  
  - Utilized advanced CAD tools (CREO, SOLIDWORKS, CATIA, Autodesk Fusion360, Inventor) to design products from refrigeration containers to buses and electronics systems.  
  - Performed Finite Element Analysis (FEA) and Computational Fluid Dynamics (CFD) using ANSYS workbench and APDL to ensure structural and thermal integrity.  
  - Collaborated with international clients including Carrier Transicold, Scania, Eaton Corporation, and Cooper Lighting.
- **Career Growth:**  
  - Progressed from intern to Engineer, successfully delivering 8+ projects.

---

### Robotics Research Engineer – [LOGOS Robotics Lab](https://logos-robotics-lab.github.io/), Arizona State University  🤖🤖
*January 2025 – Present*

- **PerSPARAMA agent architecture:**
  Coupling sparse voxel transformer perception with a homeostatic self-model via bidirectional cross-attention into a joint situational latent. Self-worth modelled via dual-utility PPO with a state-conditioned exchange rate `α(t)`. A reality-alignment loss jointly updates world and self for life-like mission autonomy. → [PerSPARAMA repo](https://github.com/Ashish-Paka/PerSPARAMA-Perception-of-a-Self-worth-Perspective-Agent-with-Reality-Aligned-Mission-Autonomy)
- **CoRL 2024 collaboration with Weiwei Gu and Prof. Nakul Gopalan (ASU-SCAI):**
  Co-developing [Cross-Embodiment Skill Representation in Robotics](https://github.com/Ashish-Paka/VLM_Video_description) on the basis of the CoRL 2024 paper [*Continual Skill and Task Learning via Dialogue*](https://arxiv.org/abs/2409.03166) (arXiv:2409.03166).
- **FR3 Franka arm few-shot continual learning:**
  Deployed **ResNet-18 + ACT-LoRA** for RL and trajectory-based unsupervised skill learning. Integrated computer vision and Gemini VLM into the framework, achieving **91.4% confidence** in skill matching on the **RH20T** dataset, segmenting demonstrations into skill primitives with semantic labels.

---

### Student Rocketry Lead – [thrustMIT](https://thrustmit.in/) (Vyom & Arya, Spaceport America Cup)  🚀🚀
*August 2017 – September 2020 · India's no.1 Student Rocketry Team*

- **Role:** Senior Structures and Composites Member (Lead, 2017 – 2020).
- **Achievements:**
  - Led structural design and in-workshop manufacturing of rocket bodies for **Project Vyom** and **Project Arya**, launched from Spaceport America.
  - Awarded the **Spot Award for Design** (Project Arya, Spaceport America Cup 2019).
  - Patent holder — [Patent No. 506725](https://drive.google.com/file/d/1STNVaIr7tddPdVohORomGUYZq30x6cEv/view?usp=sharing) ("A System for Ejection") — a reliable pressurized canister-based ejection system.

---

### Internships & Additional Projects
- **BHEL, Hyderabad (June 2018):**  
  - Developed a Plain Plug Gauge tool for gas and steam turbine manufacturing design.
- **Total Knee Replacement Implant Simulation (June 2020):**  
  - Designed and simulated orthopedic implants using advanced modeling techniques.

---

## 📜 Certifications & Continuing Education

- **Self-Driving and ROS 2 – Odometry & Control** (Antonio Brandi, Udemy)  
- **Self-Driving and ROS 2 – Map & Localization** (Antonio Brandi, Udemy)  
- **Robotics and ROS 2 – Manipulators** (Antonio Brandi, Udemy)  
- **Introduction to Programming with MATLAB** (Vanderbilt University)  
- **The Complete Python Bootcamp from Zero to Hero in Python** (Jose Portilla, Udemy)  
- **Autodesk Fusion 360 Integrated CAD/CAM/CAE** (Autodesk Inc.)  
- **Introduction to Digital Manufacturing with Autodesk Fusion 360** (Autodesk Inc.)  
- **Digital Manufacturing and Design** (University at Buffalo, SUNY)  
- **Material Science: 10 Things Every Engineer Should Know** (University of California, Davis)

---


## 🛠 **Software & Tools**

### **Programming & Development**

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white) ![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white) ![C](https://img.shields.io/badge/C-A8B9CC?style=for-the-badge&logo=c&logoColor=black) ![Java](https://img.shields.io/badge/Java-007396?style=for-the-badge&logo=openjdk&logoColor=white) ![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=for-the-badge&logo=Mathworks&logoColor=white) ![CUDA](https://img.shields.io/badge/CUDA-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![Embedded C](https://img.shields.io/badge/Embedded%20C-3949AB?style=for-the-badge&logo=arm&logoColor=white) ![Bash](https://img.shields.io/badge/Bash-121011?style=for-the-badge&logo=gnu-bash&logoColor=white) ![LaTeX](https://img.shields.io/badge/LaTeX-008080?style=for-the-badge&logo=latex&logoColor=white)

### **Robotics, ROS & Simulation**

![ROS 1](https://img.shields.io/badge/ROS%201-22314E?style=for-the-badge&logo=ros&logoColor=white) ![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-22314E?style=for-the-badge&logo=ros&logoColor=white) ![Catkin](https://img.shields.io/badge/Catkin-22314E?style=for-the-badge&logo=ros&logoColor=white) ![Nav2](https://img.shields.io/badge/Nav2-1B4F8C?style=for-the-badge&logo=ros&logoColor=white) ![MoveIt](https://img.shields.io/badge/MoveIt-512BD4?style=for-the-badge&logo=robotframework&logoColor=white) ![SLAM Toolbox](https://img.shields.io/badge/SLAM%20Toolbox-4A90E2?style=for-the-badge&logo=ros&logoColor=white) ![RTAB-Map](https://img.shields.io/badge/RTAB--Map-2E7D32?style=for-the-badge&logo=ros&logoColor=white) ![Gazebo](https://img.shields.io/badge/Gazebo-EB4034?style=for-the-badge&logo=gazebo&logoColor=white) ![NVIDIA Isaac Sim](https://img.shields.io/badge/NVIDIA%20Isaac%20Sim-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![CARLA](https://img.shields.io/badge/CARLA-1F3A93?style=for-the-badge&logo=carthrottle&logoColor=white) ![CoppeliaSim (V-REP)](https://img.shields.io/badge/CoppeliaSim%20(V--REP)-005CA9?style=for-the-badge&logo=robotframework&logoColor=white) ![PyBullet](https://img.shields.io/badge/PyBullet-FF9900?style=for-the-badge&logo=python&logoColor=white) ![MuJoCo](https://img.shields.io/badge/MuJoCo-5A5A5A?style=for-the-badge&logo=robotframework&logoColor=white) ![Webots](https://img.shields.io/badge/Webots-006699?style=for-the-badge&logo=robotframework&logoColor=white) ![Gymnasium](https://img.shields.io/badge/Gymnasium-0081A7?style=for-the-badge&logo=openai&logoColor=white) ![OpenAI Gym](https://img.shields.io/badge/OpenAI%20Gym-0081A7?style=for-the-badge&logo=openai&logoColor=white) ![URDF](https://img.shields.io/badge/URDF-22314E?style=for-the-badge&logo=ros&logoColor=white) ![TF2](https://img.shields.io/badge/TF2-22314E?style=for-the-badge&logo=ros&logoColor=white) ![RViz](https://img.shields.io/badge/RViz-22314E?style=for-the-badge&logo=ros&logoColor=white)

### **AI, Perception & ML Frameworks**

![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white) ![TensorFlow](https://img.shields.io/badge/TensorFlow-FF6F00?style=for-the-badge&logo=tensorflow&logoColor=white) ![Hugging Face](https://img.shields.io/badge/Hugging%20Face-FFD21E?style=for-the-badge&logo=huggingface&logoColor=black) ![TensorRT](https://img.shields.io/badge/TensorRT-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![ONNX](https://img.shields.io/badge/ONNX-005CED?style=for-the-badge&logo=onnx&logoColor=white) ![Weights & Biases](https://img.shields.io/badge/Weights%20%26%20Biases-FFBE00?style=for-the-badge&logo=weightsandbiases&logoColor=black) ![TensorBoard](https://img.shields.io/badge/TensorBoard-FF6F00?style=for-the-badge&logo=tensorflow&logoColor=white) ![scikit-learn](https://img.shields.io/badge/scikit--learn-F7931E?style=for-the-badge&logo=scikit-learn&logoColor=white) ![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white) ![Pandas](https://img.shields.io/badge/Pandas-150458?style=for-the-badge&logo=pandas&logoColor=white) ![FAISS](https://img.shields.io/badge/FAISS-3B5998?style=for-the-badge&logo=meta&logoColor=white) ![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white) ![Open3D](https://img.shields.io/badge/Open3D-2A6FD9?style=for-the-badge&logo=python&logoColor=white) ![Point Cloud Library](https://img.shields.io/badge/PCL-2A6FD9?style=for-the-badge&logo=ros&logoColor=white) ![MediaPipe](https://img.shields.io/badge/MediaPipe-0097A7?style=for-the-badge&logo=google&logoColor=white) ![ResNet](https://img.shields.io/badge/ResNet--18-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white) ![Vision Transformers](https://img.shields.io/badge/Vision%20Transformers-764ABC?style=for-the-badge&logo=pytorch&logoColor=white) ![ACT-LoRA](https://img.shields.io/badge/ACT--LoRA-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white) ![PPO / RL](https://img.shields.io/badge/PPO%20%2F%20RL-0081A7?style=for-the-badge&logo=openai&logoColor=white) ![LiDAR SLAM](https://img.shields.io/badge/LiDAR--SLAM-4A90E2?style=for-the-badge&logo=ros&logoColor=white)

### **LLMs, VLMs & Voice**

![OpenAI GPT](https://img.shields.io/badge/OpenAI%20GPT-412991?style=for-the-badge&logo=openai&logoColor=white) ![Anthropic Claude](https://img.shields.io/badge/Anthropic%20Claude-D97757?style=for-the-badge&logo=anthropic&logoColor=white) ![Google Gemini VLM](https://img.shields.io/badge/Google%20Gemini-4285F4?style=for-the-badge&logo=googlegemini&logoColor=white) ![Whisper STT](https://img.shields.io/badge/Whisper%20STT-412991?style=for-the-badge&logo=openai&logoColor=white)

### **Embedded, Edge & Hardware**

![NVIDIA Jetson Orin](https://img.shields.io/badge/Jetson%20Orin%20Nano-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![NVIDIA Jetson AGX](https://img.shields.io/badge/Jetson%20AGX%20Xavier-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![JetPack](https://img.shields.io/badge/JetPack%206.x-76b900?style=for-the-badge&logo=nvidia&logoColor=white) ![Arduino](https://img.shields.io/badge/Arduino%20Mega-00979D?style=for-the-badge&logo=arduino&logoColor=white) ![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=raspberry-pi&logoColor=white) ![Stereolabs ZED](https://img.shields.io/badge/Stereolabs%20ZED-0090E0?style=for-the-badge&logo=intel&logoColor=white) ![Unitree LiDAR](https://img.shields.io/badge/Unitree%20L1%20LiDAR-FF6600?style=for-the-badge&logo=lidar&logoColor=white) ![Franka FR3](https://img.shields.io/badge/Franka%20FR3-262626?style=for-the-badge&logo=robotframework&logoColor=white) ![TurtleBot3](https://img.shields.io/badge/TurtleBot3-22314E?style=for-the-badge&logo=ros&logoColor=white) ![sUAS Drone](https://img.shields.io/badge/sUAS%20Pilot%20(USA)-1E88E5?style=for-the-badge&logo=dji&logoColor=white)

### **CAD & Mechanical / CAE**

![PTC Creo](https://img.shields.io/badge/PTC%20Creo-00599C?style=for-the-badge&logo=PTC&logoColor=white) ![SolidWorks](https://img.shields.io/badge/SolidWorks-E2231A?style=for-the-badge&logo=dassaultsystemes&logoColor=white) ![CATIA](https://img.shields.io/badge/CATIA-005CA9?style=for-the-badge&logo=dassaultsystemes&logoColor=white) ![Autodesk Inventor](https://img.shields.io/badge/Inventor-FF6600?style=for-the-badge&logo=autodesk&logoColor=white) ![Fusion 360](https://img.shields.io/badge/Fusion%20360-FAA41A?style=for-the-badge&logo=autodesk&logoColor=white) ![ANSYS](https://img.shields.io/badge/ANSYS-FF9900?style=for-the-badge&logo=ansys&logoColor=white) ![ANSYS Fluent (CFD)](https://img.shields.io/badge/ANSYS%20Fluent%20(CFD)-FF9900?style=for-the-badge&logo=ansys&logoColor=white) ![COMSOL](https://img.shields.io/badge/COMSOL-5A5A5A?style=for-the-badge&logo=comsol&logoColor=white) ![OpenRocket](https://img.shields.io/badge/OpenRocket-2D5BE3?style=for-the-badge&logo=rocket&logoColor=white)

### **AR / VR & 3D**

![Unity](https://img.shields.io/badge/Unity-100000?style=for-the-badge&logo=unity&logoColor=white) ![Meta Quest](https://img.shields.io/badge/Meta%20Quest%20(MQDH)-1877F2?style=for-the-badge&logo=meta&logoColor=white) ![WebXR](https://img.shields.io/badge/WebXR-FF6F00?style=for-the-badge&logo=webxr&logoColor=white) ![Blender](https://img.shields.io/badge/Blender-F5792A?style=for-the-badge&logo=blender&logoColor=white)

### **DevOps, Version Control & Tooling**

![Git](https://img.shields.io/badge/Git-F05032?style=for-the-badge&logo=git&logoColor=white) ![GitHub](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white) ![GitHub Actions](https://img.shields.io/badge/GitHub%20Actions-2088FF?style=for-the-badge&logo=github-actions&logoColor=white) ![Docker](https://img.shields.io/badge/Docker-2496ED?style=for-the-badge&logo=docker&logoColor=white) ![VS Code](https://img.shields.io/badge/VS%20Code-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white) ![Conda](https://img.shields.io/badge/Conda-44A833?style=for-the-badge&logo=anaconda&logoColor=white) ![tmux](https://img.shields.io/badge/tmux-1BB91F?style=for-the-badge&logo=tmux&logoColor=white) ![Jupyter](https://img.shields.io/badge/Jupyter-F37626?style=for-the-badge&logo=jupyter&logoColor=white)

### **Operating Systems**

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white) ![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black) ![Windows](https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white) ![macOS](https://img.shields.io/badge/macOS-000000?style=for-the-badge&logo=apple&logoColor=white) ![Android](https://img.shields.io/badge/Android-3DDC84?style=for-the-badge&logo=android&logoColor=white) ![iOS](https://img.shields.io/badge/iOS-000000?style=for-the-badge&logo=apple&logoColor=white)

---

## 📈 Let's Connect!

- **🌐 Portfolio:** [ashish-paka.netlify.app](https://ashish-paka.netlify.app/)
- **💻 GitHub:** [Ashish-Paka](https://github.com/Ashish-Paka)
- **💼 LinkedIn:** [ashish-autonomousrobotics](https://linkedin.com/in/ashish-autonomousrobotics)
- **📧 Email:** [ashishpaka1998@gmail.com](mailto:ashishpaka1998@gmail.com)

*Together, let's advance robotics through interdisciplinary innovation—combining robust control, intelligent perception, and breakthrough mechanical design to shape the future of autonomous systems!* 🚀🤖✨

---
