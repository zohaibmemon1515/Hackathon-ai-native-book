import React from 'react';
import { motion } from 'framer-motion';
import styles from './HeroSection.module.css';

function HeroSection() {
  return (
    <motion.div
      className={styles.heroBanner}
      initial={{ opacity: 0, y: 50 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.8 }}
    >
      <div className={styles.heroContent}>
        <motion.h1
          className={styles.heroTitle}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.5, duration: 0.8 }}
        >
          Physical AI & Humanoid Robotics
        </motion.h1>
        <motion.p
          className={styles.heroTagline}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.7, duration: 0.8 }}
        >
          Explore the future of embodied intelligence and robotic interaction.
        </motion.p>
        <motion.div
          className={styles.buttons}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.9, duration: 0.8 }}
        >
          <a className="button button--secondary button--lg" href="/docs/intro">
            Start Learning
          </a>
        </motion.div>
      </div>
      <motion.div
        className={styles.heroImageContainer}
        initial={{ opacity: 0, scale: 0.8 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ delay: 0.3, duration: 1.0 }}
      >
        {/* Placeholder for humanoid robot illustration */}
        <img
          src="/img/robot-illustration.png" // This image needs to be created or sourced
          alt="Humanoid Robot"
          className={styles.heroImage}
        />
      </motion.div>
    </motion.div>
  );
}

export default HeroSection;