import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const MODULES = [
  {
    id: 1,
    level: 'BEGINNER',
    title: 'Neural Architecture',
    desc: 'Understanding the core AI decision-making frameworks',
    active: true
  },
  {
    id: 2,
    level: 'INTERMEDIATE',
    title: 'Sensor Fusion',
    desc: 'Multi-modal perception and real-time data integration',
    active: false
  },
  {
    id: 3,
    level: 'INTERMEDIATE',
    title: 'Motion Planning',
    desc: 'Dynamic trajectory optimization for bipedal locomotion',
    active: false
  },
  {
    id: 4,
    level: 'ADVANCED',
    title: 'Force Control',
    desc: 'Impedance control and haptic feedback loops',
    active: false
  }
];

function SensorNode({ angle, label, value, radius = 100 }) {
  const rad = (angle * Math.PI) / 180;
  const x = Math.cos(rad) * radius;
  const y = Math.sin(rad) * radius;
  
  return (
    <>
      <div 
        className={styles.node} 
        style={{ 
          top: `calc(50% + ${y}px)`, 
          left: `calc(50% + ${x}px)` 
        }}
      >
        <div className={styles.nodeLabel}>{label}</div>
        <div className={styles.nodeValue}>{value}</div>
      </div>
      <div 
        className={styles.connector}
        style={{
          width: radius,
          transform: `rotate(${angle}deg)`
        }}
      />
    </>
  );
}

export default function LearningPath() {
  const [activeId, setActiveId] = useState(1);

  return (
    <section className={styles.sectionContainer}>
      <div className="container">
        <div className={styles.header}>
          <span className={styles.subHeader}>CHAPTER_OBJECTIVES</span>
          <h2 className={styles.title}>LEARNING PATH</h2>
        </div>

        <div className="row">
          {/* Left: Card List */}
          <div className="col col--6">
            <div className={styles.cardList}>
              {MODULES.map((mod) => (
                <div 
                  key={mod.id} 
                  className={clsx(styles.card, activeId === mod.id && styles.active)}
                  onClick={() => setActiveId(mod.id)}
                >
                  <div className={styles.cardHeader}>
                    <div className={styles.statusIcon}>
                      {activeId === mod.id ? '◉' : '○'}
                    </div>
                    <span className={styles.cardNumber}>0{mod.id}</span>
                    <span className={clsx(styles.cardBadge, styles[`badge${mod.level.charAt(0) + mod.level.slice(1).toLowerCase()}`])}>
                      {mod.level}
                    </span>
                  </div>
                  <h3 className={styles.cardTitle}>{mod.title}</h3>
                  <p className={styles.cardDesc}>{mod.desc}</p>
                </div>
              ))}
            </div>
          </div>

          {/* Right: Visualization */}
          <div className="col col--6">
            <div className={styles.visContainer}>
              <div className={styles.visLabel}>LIVE_SENSOR_MAP</div>
              
              <div className={styles.sensorMap}>
                <div className={styles.centralNode}></div>
                <div className={styles.nodeValue} style={{top: '55%', color: '#00f3ff'}}>SYSTEM_ONLINE</div>
                
                {/* Orbital Rings */}
                <div className={clsx(styles.orbitRing, styles.ring1)}></div>
                <div className={clsx(styles.orbitRing, styles.ring2)}></div>
                <div className={clsx(styles.orbitRing, styles.ring3)}></div>

                {/* Nodes */}
                <SensorNode angle={30} label="LIDAR_360" value="99.2%" radius={130} />
                <SensorNode angle={90} label="FORCE_FB" value="94.5%" radius={130} />
                <SensorNode angle={150} label="MOTOR_R" value="89.1%" radius={130} />
                <SensorNode angle={210} label="MOTOR_L" value="87.3%" radius={130} />
                <SensorNode angle={270} label="IMU_6DOF" value="97.8%" radius={130} />
                <SensorNode angle={330} label="VISION_SYS" value="98.7%" radius={130} />
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
