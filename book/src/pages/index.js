import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import "../css/custom.css"; // The central CSS file for all styles



export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  // The style is now minimal, relying on CSS variables defined in custom.css
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Build intelligent agents and humanoid robots with hands-on projects"
      // No inline styles needed here for background, handled by global CSS
    >
      {/* HERO SECTION */}
      <header className="heroBanner">
        {/* Subtle background glow/overlay (keep inline for specific positioning) */}
        <div
            style={{
                position: 'absolute',
                top: '-50px',
                left: '-50px',
                width: '200px',
                height: '200px',
                background: 'var(--color-accent-a)',
                opacity: 0.1,
                borderRadius: '50%',
                filter: 'blur(100px)',
                zIndex: 0,
            }}
        />

        {/* Robot illustration on the right */}
        <img
          src="/img/robot-illustration.png" 
          alt="Robot Illustration"
          className="robot-illustration"
          style={{
            position: "absolute",
            right: 0,
            bottom: 0,
            maxHeight: "90%", 
            pointerEvents: "none",
            zIndex: 0,
          }}
        />

        <div className="container" style={{ position: "relative", zIndex: 1 }}>
          <h1 className="heroTitle">
            Physical AI & Humanoid Robotics
          </h1>
          <p className="heroSubtitle">
            Master Physical AI, Robotics & Humanoid Systems — Step by Step
          </p>

          <div className="buttons" style={{ marginTop: '2.5rem' }}>
            {/* Primary Button */}
            <Link
              className="button button--lg button-cta-primary"
              to="/docs/category/introduction"
              style={{ marginRight: "1rem" }}
            >
              Start Learning →
            </Link>
            
            {/* Secondary Button */}
            <Link
              className="button button--lg button-cta-secondary"
              to="/blog"
            >
              Visit Blog
            </Link>
          </div>
        </div>
      </header>

      {/* FEATURES SECTION */}
      <main>
        <div className="container" style={{ padding: '100px 20px' }}>
            <h2 style={{ 
                textAlign: 'center', 
                marginBottom: '5rem', 
                fontSize: '2.8rem', 
            }}>
                Key Focus Areas
            </h2>
            <section className="features-grid">
              {/* Feature Card 1 */}
              <div className="featureCard">
                <h3>🤖 Humanoid Robotics</h3>
                <p style={{ color: 'var(--color-text-secondary)' }}>
                  Learn to control, program, and simulate **humanoid robots** for
                  real-world AI applications.
                </p>
              </div>

              {/* Feature Card 2 */}
              <div className="featureCard">
                <h3>🧠 Embodied Intelligence</h3>
                <p style={{ color: 'var(--color-text-secondary)' }}>
                  Understand how AI agents perceive, reason, and act in the **physical
                  world** with advanced sensors.
                </p>
              </div>

              {/* Feature Card 3 */}
              <div className="featureCard">
                <h3>⚙️ Hands-On Projects</h3>
                <p style={{ color: 'var(--color-text-secondary)' }}>
                  Build practical projects and capstones with **step-by-step**
                  tutorials in simulation and hardware.
                </p>
              </div>

              {/* Feature Card 4 */}
              <div className="featureCard">
                <h3>📚 Advanced Learning</h3>
                <p style={{ color: 'var(--color-text-secondary)' }}>
                  Explore advanced AI topics, robotics algorithms, and system **integration** strategies.
                </p>
              </div>
            </section>
        </div>
        

        {/* --- NEW SECTION: FINAL CTA --- */}
        <section className="cta-banner">
            <div className="container">
                <h2>Ready to Step into the Future of Robotics?</h2>
                <p style={{ fontSize: '1.2rem', margin: '1rem auto 2rem', color: '#FFF8E1' }}>
                    Start your journey with the most comprehensive guide to Physical AI today.
                </p>
                <Link
                    className="button button--lg button-cta-primary"
                    to="/docs/category/introduction"
                    style={{ 
                        backgroundColor: 'white', // Overrides primary CTA color for contrast on orange background
                        color: 'var(--color-dark-bg)',
                        boxShadow: '0 4px 20px rgba(0, 0, 0, 0.5)' 
                    }}
                >
                    Enroll Now →
                </Link>
            </div>
        </section>
      </main>
    </Layout>
  );
}