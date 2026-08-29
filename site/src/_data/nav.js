/** Course IA: topics with separately numbered lectures (guests are not a topic). */
module.exports = {
  top: [
    { title: "Overview", url: "/" },
    { title: "Syllabus", url: "/syllabus/" },
    { title: "Setup", url: "/setup/" },
    { title: "Practices", url: "/practices/" },
    { title: "Demos", url: "/demos/" },
  ],
  topics: [
    {
      id: "01",
      title: "Intro & simulation",
      url: "/topics/01-intro/",
      section: "I — Intro & classical",
      lectures: [
        {
          n: 1,
          title: "What is control? Feedback, objectives, architecture; brief kinematics & dynamics recap",
          slides: "/lectures/01-intro/",
        },
        {
          n: 2,
          title: "Simulation: ODEs, MuJoCo / software stack, sensing & actuation",
          slides: "/lectures/01-intro/",
        },
      ],
      practice: {
        title: "State-space toys → MuJoCo; open-loop sim; plot",
        url: "/practices/#week-1",
      },
    },
    {
      id: "02",
      title: "Classical control",
      url: "/topics/02-classical/",
      section: "I — Intro & classical",
      lectures: [
        {
          n: 3,
          title: "Velocity / Jacobian-based control (tracking, singularities)",
        },
        {
          n: 4,
          title: "Linear control: PD/PID, full-state feedback",
        },
      ],
      practice: {
        title: "Manipulator: IK + ID (Jacobian / PD / grav vs ID)",
        url: "/practices/#week-2",
      },
    },
    {
      id: "03",
      title: "Dynamic control & LQR",
      url: "/topics/03-lqr/",
      section: "II — Optimal control",
      lectures: [
        {
          n: 5,
          title: "Inverse dynamics & gravity compensation (+ light Lyapunov / energy)",
        },
        {
          n: 6,
          title: "LQR (+ trajectory-optimization sketch)",
        },
      ],
      practice: {
        title: "LQR on cart-pole & quadrotor; TH1 start",
        url: "/practices/#week-3",
      },
    },
    {
      id: "04",
      title: "Receding horizon & sampling",
      url: "/topics/04-mpc/",
      section: "II — Optimal control",
      lectures: [
        { n: 7, title: "Model predictive control (MPC)" },
        { n: 8, title: "Model predictive path integral (MPPI)" },
      ],
      practice: {
        title: "TH1: quadrotor traj opt + constrained MPC",
        url: "/practices/#week-4",
      },
    },
    {
      id: "05",
      title: "Imitation learning",
      url: "/topics/05-imitation/",
      section: "III — Learning",
      lectures: [
        {
          n: 9,
          title: "Behavior cloning: setup, data, supervised policies, failure modes",
        },
        {
          n: 10,
          title: "DAgger: algorithm, distribution shift, practice patterns, limits",
        },
      ],
      practice: {
        title: "TH2 kickoff: BC + DAgger",
        url: "/practices/#week-5",
      },
    },
    {
      id: "06",
      title: "Reinforcement learning",
      url: "/topics/06-rl/",
      section: "III — Learning",
      lectures: [
        {
          n: 11,
          title: "RL basics: MDP, observations, actions, rewards, policy",
        },
        {
          n: 12,
          title: "Cases: quadruped tracking + rewards; humanoid IL; short course wrap-up",
        },
      ],
      practice: {
        title: "RL experiments + project progress",
        url: "/practices/#week-6",
      },
    },
  ],
  resources: [
    { title: "Syllabus PDF", url: "/syllabus.pdf", external: false },
    {
      title: "GitHub",
      url: "https://github.com/simeon-ned/forc",
      external: true,
    },
    {
      title: "Instructor",
      url: "https://simeon-ned.github.io",
      external: true,
    },
  ],
};
