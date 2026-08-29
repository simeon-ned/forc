# Syllabus Template

• Course name: Fundamentals of Robot Control

• Subject area: Robotics

P.1 Short Description
This introductory course focuses on control over robotic systems, primarily fully actuated ones. Students will learn the fundamentals of analysis and control over robotic systems described by linear and nonlinear models. Key concepts include state space modeling, analysis of linear and nonlinear systems, linear control (PD, PID, pole placement, etc.), and feedback linearization.

P.2 Intended Learning Outcomes (ILOs)
P.2.1 What is the purpose of this course?

The main purpose of this course is to provide students with a solid foundation in robot control theory and practice, enabling them to analyze and design control systems for various robotic applications.

P.2.2 ILOs defined at three levels
Level 1: What concepts should a student understand/know/remember/explain at the end of the course?
By the end of the course, the students should be able to understand:
- State space modeling of robotic systems
- Stability analysis of linear and nonlinear systems
- Linear control techniques (PD, PID, pole placement)
- Feedback linearization and gravity compensation
- The structure of differential equations in robotics
- The main sources of nonlinearities in robotic systems

Level 2: What basic practical skills should a student be able to perform at the end of the course?
By the end of the course, the students should be able to:
- Develop state space models for robotic systems
- Analyze the stability of linear and nonlinear robotic systems
- Design linear controllers for robotic systems
- Implement feedback linearization techniques
- Simulate robotic systems using Python and Google Colab
- Perform basic stability analysis of robots given their dynamical model

Level 3: What complex comprehensive skills should a student be able to apply in real-life scenarios?
By the end of the course, the students should be able to:
- Design and implement control systems for various types of robots
- Analyze and optimize the performance of robotic control systems
- Apply different control algorithms for motion control of diverse robots
- Integrate linear and nonlinear control techniques in complex robotic applications
- Evaluate and select appropriate control strategies based on specific robotic system requirements
- Troubleshoot and improve existing robotic control systems

P.3 Course sections
The main sections of the course and approximate hour distribution between them is as follows:

Table P.3: Course Sections
Section | Section Title
--- | ---
1  | Introduction and Modeling
2  | Analysis and Control of Linear Systems
3  | Analysis and Control over Nonlinear Systems
4  | Other Topics
5  | Exams and Review

P.3.1.1 Section 1
Section title: Introduction and Modeling
Topics covered in this section:
1.1 Concept of dynamical system
1.2 Model introduction
1.3 State space representation
1.4 Linear and nonlinear systems
1.5 Solutions of ODE and simulation

What forms of evaluation were used to test students' performance in this section?
Form | Yes/No
--- | ---
Development of individual parts of software product code | Yes
Homework and group projects | Yes
Midterm evaluation | No
Testing (written or computer based) | Yes
Reports | No
Essays | No
Oral polls | Yes
Discussions | Yes

• Typical questions for seminar classes (labs) within this section (Appendix):
1. Define a state space model for a simple pendulum.
2. Explain the difference between linear and nonlinear systems in robotics.
3. Implement a simulation of a basic robotic arm using Python.
4. Describe the process of solving ODEs for a robotic system.
5. Compare different numerical methods for simulating robotic systems.
6. Explain the concept of state in the context of robotic systems.
7. Derive the equations of motion for a 2-DOF planar robot.
8. Discuss the importance of modeling in robotic control.
9. Implement a state space model for a DC motor in Python.
10. Explain the relationship between transfer functions and state space models.

• Typical questions for ongoing performance evaluation within this section (Appendix):
1. What are the key components of a state space model?
2. How do you determine if a system is linear or nonlinear?
3. What are the advantages of using state space representation in robotics?
4. Explain the concept of equilibrium points in the context of robotic systems.
5. How does simulation help in understanding robotic system behavior?
6. What are the limitations of linear models in representing real robotic systems?
7. Describe the process of linearizing a nonlinear robotic system.
8. What is the significance of initial conditions in ODE solutions for robots?
9. How do you choose appropriate state variables for a robotic system?
10. Explain the concept of controllability in state space models.

• Questions for exam preparation within this section (Appendix):
1. Derive the state space model for a simple mass-spring-damper system.
2. Compare and contrast linear and nonlinear systems in the context of robotics.
3. Explain the process of converting a transfer function to a state space model.
4. Discuss the importance of choosing appropriate state variables in robotic modeling.
5. Describe the steps to simulate a nonlinear robotic system using Python.
6. What are the key challenges in modeling complex robotic systems?
7. Explain the concept of phase space and its relevance in robotic system analysis.
8. How does feedback affect the dynamics of a robotic system?
9. Discuss the role of Jacobian matrices in robotic system modeling.
10. Explain the concept of decoupling in multi-input multi-output robotic systems.

P.3.1.2 Section 2
Section title: Analysis and Control of Linear Systems
Topics covered in this section:
2.1 Introduction to stability
2.2 Concept of full state feedback
2.3 PD and PID control
2.4 Pole placement
2.5 Linear Quadratic Regulator (LQR)

What forms of evaluation were used to test students' performance in this section?
Form | Yes/No
--- | ---
Development of individual parts of software product code | Yes
Homework and group projects | Yes
Midterm evaluation | Yes
Testing (written or computer based) | Yes
Reports | No
Essays | No
Oral polls | Yes
Discussions | Yes

• Typical questions for seminar classes (labs) within this section (Appendix):
1. Implement a PID controller for a simple robotic arm in Python.
2. Analyze the stability of a given linear system using eigenvalue analysis.
3. Design a full state feedback controller for a 2-DOF planar robot.
4. Compare the performance of PD and PID controllers for a specific robotic system.
5. Implement pole placement technique for a given linear system.
6. Discuss the trade-offs in choosing poles for a robotic control system.
7. Design an LQR controller for a simple inverted pendulum.
8. Analyze the robustness of different linear control techniques.
9. Implement a state observer for a linear robotic system.
10. Discuss the limitations of linear control techniques in real robotic applications.

• Typical questions for ongoing performance evaluation within this section (Appendix):
1. What are the conditions for stability in linear systems?
2. Explain the concept of controllability in the context of full state feedback.
3. How does integral action in PID control affect steady-state error?
4. What are the advantages and disadvantages of pole placement technique?
5. Explain the concept of the Linear Quadratic Regulator (LQR).
6. How do you choose appropriate gains for a PID controller?
7. What is the significance of the characteristic equation in stability analysis?
8. Explain the concept of observability in linear systems.
9. How does feedback affect the poles of a closed-loop system?
10. Discuss the role of the state transition matrix in linear system analysis.

• Questions for exam preparation within this section (Appendix):
1. Derive the closed-loop transfer function for a system with full state feedback.
2. Analyze the stability of a given linear system using the Routh-Hurwitz criterion.
3. Design a PID controller for a given robotic system and analyze its performance.
4. Explain the process of pole placement and its limitations.
5. Derive the Riccati equation for the LQR problem.
6. Compare and contrast different linear control techniques for robotic systems.
7. Discuss the concept of controllability and its implications in robotic control.
8. Explain how integral control affects system type and steady-state error.
9. Analyze the robustness of a full state feedback controller to parameter variations.
10. Describe the process of designing an optimal LQR controller for a robotic system.

P.3.1.3 Section 3
Section title: Analysis and Control over Nonlinear Systems
Topics covered in this section:
3.1 Linearization techniques
3.2 Lyapunov stability theory
3.3 Linear control over nonlinear systems
3.4 Full feedback linearization
3.5 Partial feedback linearization

What forms of evaluation were used to test students' performance in this section?
Form | Yes/No
--- | ---
Development of individual parts of software product code | Yes
Homework and group projects | Yes
Midterm evaluation | No
Testing (written or computer based) | Yes
Reports | Yes
Essays | No
Oral polls | Yes
Discussions | Yes

• Typical questions for seminar classes (labs) within this section (Appendix):
1. Linearize a given nonlinear robotic system around an equilibrium point.
2. Implement a Lyapunov function to analyze the stability of a nonlinear system.
3. Design a linear controller for a nonlinear robotic arm and analyze its performance.
4. Apply full feedback linearization to a simple nonlinear robotic system.
5. Compare the performance of linear and nonlinear controllers for a given system.
6. Implement partial feedback linearization for an underactuated robot.
7. Analyze the region of attraction for a nonlinear control system.
8. Design a sliding mode controller for a nonlinear robotic system.
9. Implement an adaptive controller for a robot with uncertain parameters.
10. Discuss the limitations of feedback linearization in real robotic applications.

• Typical questions for ongoing performance evaluation within this section (Appendix):
1. What are the key differences between linear and nonlinear control techniques?
2. Explain the concept of Lyapunov stability and its importance in nonlinear systems.
3. How does feedback linearization transform a nonlinear system?
4. What are the conditions for applying full feedback linearization?
5. Explain the concept of zero dynamics in nonlinear control.
6. How do you choose an appropriate Lyapunov function for stability analysis?
7. What are the advantages of nonlinear control over linear control in robotics?
8. Explain the concept of input-output linearization.
9. How does partial feedback linearization differ from full feedback linearization?
10. Discuss the challenges in controlling underactuated robotic systems.

• Questions for exam preparation within this section (Appendix):
1. Derive the linearized model of a nonlinear robotic system around an equilibrium point.
2. Analyze the stability of a nonlinear system using Lyapunov's direct method.
3. Design a feedback linearizing controller for a given nonlinear robotic system.
4. Compare and contrast different nonlinear control techniques for robotic systems.
5. Explain the process of designing a sliding mode controller for a robot.
6. Discuss the concept of adaptive control and its applications in robotics.
7. Analyze the robustness of nonlinear controllers to parameter uncertainties.
8. Explain the challenges in controlling non-minimum phase nonlinear systems.
9. Describe the process of designing a backstepping controller for a robotic system.
10. Discuss the limitations and practical considerations of implementing nonlinear controllers in real robotic systems.

P.3.1.4 Section 4
Section title: Other Topics
Topics covered in this section:
4.1 State observers
4.2 Adaptive control
4.3 Robust control
4.4 Overview of advanced control techniques
4.5 Future trends in robotic control

What forms of evaluation were used to test students' performance in this section?
Form | Yes/No
--- | ---
Development of individual parts of software product code | Yes
Homework and group projects | Yes
Midterm evaluation | No
Testing (written or computer based) | Yes
Reports | Yes
Essays | No
Oral polls | Yes
Discussions | Yes

• Typical questions for seminar classes (labs) within this section (Appendix):
1. Implement a Luenberger observer for a linear robotic system.
2. Design an adaptive controller for a robot with uncertain mass properties.
3. Implement a robust controller for a robotic system with bounded uncertainties.
4. Compare the performance of different state estimation techniques for a given robotic system.
5. Design a Model Predictive Controller (MPC) for a simple robotic task.
6. Implement an Extended Kalman Filter (EKF) for state estimation in a nonlinear robotic system.
7. Analyze the robustness of different control techniques to external disturbances.
8. Implement a basic reinforcement learning algorithm for a simple robotic control task.
9. Design a fuzzy logic controller for a robotic system.
10. Discuss the potential applications of machine learning in robotic control.

• Typical questions for ongoing performance evaluation within this section (Appendix):
1. What are the key differences between state feedback and output feedback control?
2. Explain the concept of adaptive control and its advantages in robotics.
3. How does robust control deal with uncertainties in robotic systems?
4. What are the main challenges in implementing observers for nonlinear systems?
5. Explain the basic principle of Model Predictive Control (MPC).
6. How does reinforcement learning differ from traditional control techniques?
7. What are the advantages and limitations of fuzzy logic control in robotics?
8. Explain the concept of H-infinity control and its applications in robotics.
9. How do neural networks contribute to advanced robotic control systems?
10. Discuss the potential impact of quantum computing on future robotic control systems.

• Questions for exam preparation within this section (Appendix):
1. Design a state observer for a given nonlinear robotic system.
2. Analyze the stability and convergence of an adaptive control system.
3. Develop a robust control strategy for a robot operating in an uncertain environment.
4. Compare and contrast different advanced control techniques for complex robotic systems.
5. Explain the process of designing an MPC controller for a robotic manipulation task.
6. Discuss the challenges and potential solutions in controlling soft robotic systems.
7. Analyze the trade-offs between model-based and learning-based control approaches in robotics.
8. Explain how adaptive and robust control techniques can be combined for improved performance.
9. Describe the potential applications of optimal control theory in advanced robotic systems.
10. Discuss the future trends and challenges in robotic control systems.

P.3.1.5 Section 5
Section title: Exams and Review
Topics covered in this section:
5.1 Midterm exam review
5.2 Midterm exam
5.3 Final exam review
5.4 Final exam
5.5 Course wrap-up and future directions

What forms of evaluation were used to test students' performance in this section?
Form | Yes/No
--- | ---
Development of individual parts of software product code | No
Homework and group projects | No
Midterm evaluation | Yes
Testing (written or computer based) | Yes
Reports | No
Essays | No
Oral polls | Yes
Discussions | Yes

• Typical questions for seminar classes (labs) within this section (Appendix):
1. Review and solve sample problems covering linear system analysis and control.
2. Practice deriving state space models for various robotic systems.
3. Analyze the stability of given linear and nonlinear systems.
4. Design and simulate different types of controllers for robotic systems.
5. Solve problems related to feedback linearization and Lyapunov stability.
6. Review and discuss case studies of real-world robotic control applications.
7. Practice implementing control algorithms in Python.
8. Analyze and compare the performance of different control strategies.
9. Review advanced topics such as adaptive and robust control.
10. Discuss potential research directions in robotic control.

• Typical questions for ongoing performance evaluation within this section (Appendix):
1. Explain the key differences between linear and nonlinear control techniques.
2. How do you choose between different control strategies for a given robotic system?
3. What are the main challenges in implementing theoretical control concepts in real robots?
4. How does the choice of state variables affect the control system design?
5. Explain the relationship between stability, controllability, and observability.
6. How do uncertainties and disturbances impact robotic control system performance?
7. What are the trade-offs between model-based and data-driven control approaches?
8. How do you validate and test a robotic control system before real-world deployment?
9. Explain the importance of simulation in robotic control system design.
10. What are the emerging trends in robotic control that excite you the most?

• Questions for exam preparation within this section (Appendix):
1. Design a complete control system for a given robotic manipulation task.
2. Analyze the stability and performance of a nonlinear robotic system.
3. Compare and contrast different control strategies for an underactuated robot.
4. Develop a robust control strategy for a robot operating in an uncertain environment.
5. Design an adaptive controller for a robot with varying payload.
6. Analyze the observability and controllability of a given robotic system.
7. Implement and analyze a nonlinear observer for state estimation.
8. Design an optimal control strategy for a multi-robot coordination problem.
9. Analyze the stability of a robot-environment interaction control system.
10. Develop a control strategy for a soft robotic system with distributed actuation.

P.3.2 Typical questions for final assessment in this section (Appendix):
1. Design a full state feedback controller for a 3-DOF robotic manipulator.
2. Analyze the stability of a given nonlinear robotic system using Lyapunov theory.
3. Implement and compare PID and LQR controllers for a simple robotic system.
4. Design a feedback linearization controller for a nonlinear robotic system.
5. Analyze the performance of an adaptive controller for a robot with uncertain parameters.
6. Develop a robust control strategy for a robot operating in an environment with bounded disturbances.
7. Design and implement a state observer for a nonlinear robotic system.
8. Analyze the stability and performance of a sliding mode controller for a robotic system.
9. Implement and evaluate a Model Predictive Controller (MPC) for a robotic path following task.
10. Design a control system for an underactuated robotic system using partial feedback linearization.
11. Analyze the zero dynamics of a given nonlinear robotic system.
12. Implement and evaluate an Extended Kalman Filter (EKF) for state estimation in a mobile robot.
13. Design an optimal control strategy for a multi-robot formation control problem.
14. Analyze the stability of a robot-environment interaction control system using passivity theory.
15. Develop a control strategy for a soft robotic gripper with distributed actuation.
16. Implement and evaluate a reinforcement learning algorithm for a simple robotic control task.
17. Design a fuzzy logic controller for a robot operating in an uncertain environment.
18. Analyze the performance of different control strategies for a robot manipulator with flexible joints.
19. Develop a control system for a quadrotor drone performing aggressive maneuvers.
20. Implement and evaluate an adaptive neural network controller for a complex robotic system.
21. Design a hybrid control strategy combining discrete and continuous dynamics for a walking robot.
22. Analyze the stability and performance of a distributed control system for a swarm of robots.
23. Develop a control strategy for a robot manipulator interacting with a human operator.
24. Implement and evaluate an iterative learning control algorithm for a repetitive robotic task.
25. Design a control system for a robot performing a constrained motion task.
26. Analyze the performance of different trajectory generation methods for robotic motion planning.
27. Develop a control strategy for a robot manipulator with kinematic redundancy.
28. Implement and evaluate a force control scheme for a robot performing assembly tasks.
29. Design a control system for a legged robot walking on uneven terrain.
30. Analyze the stability of a teleoperation system with time delay.
31. Develop a control strategy for a robot manipulator handling objects with unknown inertial properties.
32. Implement and evaluate an adaptive impedance control scheme for human-robot collaboration.
33. Design a control system for a robot performing visual servoing tasks.
34. Analyze the performance of different collision avoidance algorithms for mobile robots.
35. Develop a control strategy for a robot manipulator performing high-speed pick-and-place operations.
36. Implement and evaluate a model-free adaptive control algorithm for a robotic system.
37. Design a control system for a robot performing dexterous manipulation tasks.
38. Analyze the stability and performance of a bioinspired control strategy for a robotic fish.
39. Develop a control strategy for a robot operating in a cluttered and dynamic environment.
40. Implement and evaluate a learning-based control algorithm for adapting to new tasks and environments.

P.3.3 The retake exam
For the retake, students will be required to complete a comprehensive exam covering all major topics of the course. The exam will consist of both theoretical questions and practical problem-solving tasks. Students may also be required to submit a project demonstrating their understanding and application of key course concepts.

P.4.1 Grades range
Table P.4.1: Course grading range
Grade | Range | Description of performance (optional)
--- | --- | ---
A. Excellent | 85-100% | Demonstrates comprehensive understanding and application of course concepts
B. Good | 70-84% | Shows good grasp of most course concepts with minor gaps
C. Satisfactory | 55-69% | Demonstrates basic understanding of core concepts
D. Fail | 0-54% | Insufficient understanding of course material

P.4.2 Course evaluation
Table P.4.2: Course grade breakdown
Type | Default points | Proposed points
--- | --- | ---
Labs/seminar classes | 20 | 20
Interim performance assessment | 30 | 30
Exams | 50 | 50

The course grade is determined as follows:
- Midterm Exam: 30%
- Final Exam: 30%
- Assignments: 20%
- Attendance and Participation: 20%

P.4.3 Recommendations for students on how to succeed in the course (optional)
- Attend all lectures and actively participate in discussions
- Complete all assignments and practice implementing concepts in Python
- Regularly review and practice solving problems from previous lectures
- Form study groups to discuss complex topics and solve challenging problems
- Utilize office hours to clarify doubts and seek additional guidance

P.5 Resources, literature and reference materials
P.5.1 Open access resources
- Control Bootcamp (YouTube playlist)
- Slotine control course (Bilibili video)
- Underactuated Robotics (MIT OpenCourseWare)

P.5.2 Closed access resources
- Robot Modeling and Control by Spong et al.
- Applied Nonlinear Control by Slotine and Li
- Robotics, Planning and Control by Siciliano et al.
- Modern Robotics by Kevin Lynch

P.5.3 Software and tools used within the course 
- Python
- Google Colab
- MATLAB (optional)
- ROS (Robot Operating System) for advanced topics