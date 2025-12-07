import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

// Module card data
const modules = [
  {
    title: 'Module 1: ROS 2',
    icon: '🤖',
    description: 'Master the robotic nervous system: nodes, topics, services, and URDF robot descriptions.',
    duration: '60 min',
    chapters: 3,
    link: '/docs/module-1-ros2',
  },
  {
    title: 'Module 2: Gazebo',
    icon: '🌐',
    description: 'Build digital twins with physics simulation, sensors, and realistic environments.',
    duration: '48 min',
    chapters: 2,
    link: '/docs/module-2-gazebo',
  },
  {
    title: 'Module 3: NVIDIA Isaac',
    icon: '🧠',
    description: 'Harness photorealistic simulation, synthetic data, VSLAM, and hardware-accelerated perception.',
    duration: '72 min',
    chapters: 4,
    link: '/docs/module-3-isaac',
  },
  {
    title: 'Module 4: Vision-Language-Action',
    icon: '💬',
    description: 'Integrate voice commands, LLM planning, and embodied AI for intelligent robot control.',
    duration: '60 min',
    chapters: 3,
    link: '/docs/module-4-vla',
  },
];

function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>
          <span className={styles.heroEmoji}>🤖</span>
          {siteConfig.title}
        </h1>
        <p className={styles.heroTagline}>{siteConfig.tagline}</p>
        <div className={styles.heroStats}>
          <div className={styles.statItem}>
            <span className={styles.statNumber}>4</span>
            <span className={styles.statLabel}>Modules</span>
          </div>
          <div className={styles.statItem}>
            <span className={styles.statNumber}>12</span>
            <span className={styles.statLabel}>Chapters</span>
          </div>
          <div className={styles.statItem}>
            <span className={styles.statNumber}>3.4</span>
            <span className={styles.statLabel}>Hours</span>
          </div>
        </div>
        <div className={styles.buttonGroup}>
          <Link
            className={`button button--primary button--lg ${styles.ctaButton}`}
            to="/docs/quick-start">
            📚 Start Reading
          </Link>
          <Link
            className={`button button--secondary button--lg ${styles.secondaryButton}`}
            to="/docs/hardware">
            🔧 Hardware Setup
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModuleCard({module}) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleIcon}>{module.icon}</div>
      <h3 className={styles.moduleTitle}>{module.title}</h3>
      <p className={styles.moduleDescription}>{module.description}</p>
      <div className={styles.moduleStats}>
        <span className={styles.moduleStat}>⏱️ {module.duration}</span>
        <span className={styles.moduleStat}>📖 {module.chapters} chapters</span>
      </div>
      <Link to={module.link} className={styles.moduleLink}>
        Explore Module →
      </Link>
    </div>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Your Learning Journey</h2>
        <p className={styles.sectionSubtitle}>
          Four progressive modules take you from ROS 2 fundamentals to advanced AI-powered robotics
        </p>
        <div className={styles.moduleGrid}>
          {modules.map((module, idx) => (
            <ModuleCard key={idx} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <div className={styles.featureGrid}>
          <div className={styles.featureItem}>
            <div className={styles.featureIcon}>🔧</div>
            <h3>Hands-On Tutorials</h3>
            <p>Run your first ROS 2 node in 30 minutes with step-by-step guides and runnable code examples.</p>
          </div>
          <div className={styles.featureItem}>
            <div className={styles.featureIcon}>🎮</div>
            <h3>Simulation Mastery</h3>
            <p>Master Gazebo physics and NVIDIA Isaac Sim for photorealistic robot simulation and testing.</p>
          </div>
          <div className={styles.featureItem}>
            <div className={styles.featureIcon}>👁️</div>
            <h3>AI Perception</h3>
            <p>Implement VSLAM, sensor fusion, and hardware-accelerated perception with Isaac ROS.</p>
          </div>
          <div className={styles.featureItem}>
            <div className={styles.featureIcon}>🗣️</div>
            <h3>Voice-Controlled Robots</h3>
            <p>Integrate LLMs and voice commands for natural language robot control and task planning.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function QuickLinksSection() {
  return (
    <section className={styles.quickLinksSection}>
      <div className="container">
        <div className={styles.quickLinksGrid}>
          <div className={styles.quickLinkCard}>
            <h3>🚀 New to Physical AI?</h3>
            <p>Start with our introduction to understand what Physical AI is and why it matters.</p>
            <Link to="/docs/intro" className="button button--outline">
              Read Introduction
            </Link>
          </div>
          <div className={styles.quickLinkCard}>
            <h3>⚡ Want to Jump In?</h3>
            <p>Follow the 30-minute quick start guide to run your first robot simulation.</p>
            <Link to="/docs/quick-start" className="button button--outline">
              Quick Start Guide
            </Link>
          </div>
          <div className={styles.quickLinkCard}>
            <h3>🛠️ Setting Up a Lab?</h3>
            <p>Review hardware requirements for workstations, edge devices, and robot options.</p>
            <Link to="/docs/hardware" className="button button--outline">
              Hardware Requirements
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description={siteConfig.tagline}>
      <HeroSection />
      <ModulesSection />
      <FeaturesSection />
      <QuickLinksSection />
    </Layout>
  );
}
