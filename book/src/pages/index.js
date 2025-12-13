import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import "../css/custom.css";

/* =========================
   FEATURE DOMAINS
========================= */
const features = [
  {
    title: "Humanoid Robotics Engineering",
    description:
      "Design and control full-scale humanoid robots with a focus on balance, locomotion, manipulation, and safe human interaction.",
    color: "accent-a",
  },
  {
    title: "Embodied Intelligence Systems",
    description:
      "Build AI agents that perceive, reason, and act within physical environments using sensors, world models, and decision policies.",
    color: "accent-b",
  },
  {
    title: "Applied Robotics Projects",
    description:
      "Work on industry-grade robotics projects spanning simulation, reinforcement learning, and real hardware deployment.",
    color: "accent-a",
  },
  {
    title: "Advanced Control & Agents",
    description:
      "Master robotics foundations including kinematics, dynamics, optimal control, MPC, and multi-agent coordination systems.",
    color: "accent-b",
  },
];

/* =========================
   AGENT FRAMEWORK
========================= */
const missionSections = [
  {
    icon: "🧠",
    title: "Agent Architecture & Reasoning",
    text:
      "Design intelligent agent stacks combining perception, planning, memory, and decision-making — from classical pipelines to LLM-augmented systems.",
  },
  {
    icon: "🌍",
    title: "Simulation-to-Reality Transfer",
    text:
      "Bridge the sim-to-real gap using domain randomization, sensor modeling, and hardware-aware control techniques.",
  },
  {
    icon: "⚡",
    title: "Real-Time Control & Dynamics",
    text:
      "Implement kinematics, dynamics, and optimal control to enable stable, agile, and responsive robotic motion.",
  },
];

/* =========================
   WHO THIS IS FOR
========================= */
const audience = [
  {
    title: "Robotics Engineers",
    text:
      "Engineers looking to design, control, and deploy real-world robotic systems using modern AI techniques.",
  },
  {
    title: "AI Researchers & Students",
    text:
      "Learners aiming to understand embodied intelligence, agent-based systems, and robotics research foundations.",
  },
  {
    title: "Advanced Developers",
    text:
      "Developers transitioning from software AI into physical systems, robotics simulation, and control pipelines.",
  },
];

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A deep-technical platform for building intelligent embodied agents and humanoid robots"
    >
      {/* ================= HERO ================= */}
      <header className="heroBanner">

        <div className="container hero-content-wrapper">
          <h1 className="heroTitle text-gradient">
            Physical AI & Humanoid Robotics
          </h1>

          <p className="heroSubtitle">
            A deep-technical learning platform for building{" "}
            <strong>intelligent embodied agents</strong> and{" "}
            <strong>human-scale robotic systems</strong> — from simulation to
            real-world deployment.
          </p>

          <Link
            className="button button-cta-primary"
            to="/docs/category/introduction"
          >
            Begin the Curriculum →
          </Link>
        </div>
      </header>

      <main>
        {/* ================= CORE DOMAINS ================= */}
        <section className="section-padding">
          <h2 className="section-title text-gradient">
            Core Research & Engineering Domains
          </h2>

          <div className="features-grid">
            {features.map((feature, idx) => (
              <div
                key={idx}
                className={`featureCard featureCard--${feature.color}`}
              >
                <h3>{feature.title}</h3>
                <p className="text-secondary-color">
                  {feature.description}
                </p>
              </div>
            ))}
          </div>
        </section>

        {/* ================= AGENT FRAMEWORK ================= */}
        <section className="mission-section">
          <div className="container">
            <h2 className="section-title white-text">
              The Agent Development Framework
            </h2>

            <div className="mission-grid">
              {missionSections.map((item, index) => (
                <div key={index} className="mission-card">
                  <span className="mission-icon">{item.icon}</span>
                  <h3>{item.title}</h3>
                  <p>{item.text}</p>
                </div>
              ))}
            </div>
          </div>
        </section>

        {/* ================= WHO THIS IS FOR ================= */}
        <section className="section-padding">
          <h2 className="section-title text-gradient">
            Who This Platform Is For
          </h2>

          <div className="features-grid">
            {audience.map((item, index) => (
              <div key={index} className="featureCard">
                <h3>{item.title}</h3>
                <p className="text-secondary-color">{item.text}</p>
              </div>
            ))}
          </div>
        </section>

        {/* ================= CTA ================= */}
        <section className="cta-banner">
          <div className="container">
            <h2>Build the Next Generation of Intelligent Machines</h2>
            <p>
              Follow a structured, hands-on curriculum focused on real robotics
              systems, modern AI agents, and practical engineering workflows.
            </p>

            <Link
              className="button button-cta-inverted"
              to="/docs/category/introduction"
            >
              Enroll in the Program →
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
