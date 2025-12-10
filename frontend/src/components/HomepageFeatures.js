import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Humanoid Robotics Book',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        A comprehensive textbook covering the essential technologies and concepts 
        needed to design, simulate, and deploy humanoid robots.
      </>
    ),
  },
  {
    title: 'Four Core Modules',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Explore ROS 2, Gazebo & Unity, NVIDIA Isaac, and Vision-Language-Action (VLA) 
        - the foundational technologies for humanoid robotics.
      </>
    ),
  },
  {
    title: 'Interactive Learning',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Engage with interactive content, simulations, and a RAG chatbot 
        to enhance your learning experience.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}