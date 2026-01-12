import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '2ca'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '08e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '744'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '40e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '943'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '921'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'a3c'),
    exact: true
  },
  {
    path: '/auth',
    component: ComponentCreator('/auth', '128'),
    exact: true
  },
  {
    path: '/login',
    component: ComponentCreator('/login', 'ad5'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', '293'),
    exact: true
  },
  {
    path: '/what-you-learn',
    component: ComponentCreator('/what-you-learn', '1b5'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '8f7'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '8d8'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '004'),
            routes: [
              {
                path: '/docs/ai-robot-brain/',
                component: ComponentCreator('/docs/ai-robot-brain/', '3ee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/A1-costmap-configuration',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/A1-costmap-configuration', '624'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/A2-planners-behavior-trees',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/A2-planners-behavior-trees', 'e82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/A3-reinforcement-learning',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/A3-reinforcement-learning', 'aaf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/A4-sim-to-real',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/A4-sim-to-real', '312'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/pretrained/',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/pretrained/', '5bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/B1-introduction-perception',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/B1-introduction-perception', '4d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/B2-sensor-types',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/B2-sensor-types', 'f37'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/B3-slam-navigation-intro',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/B3-slam-navigation-intro', '067'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/diagrams/palette',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/diagrams/palette', '70d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/refresher-gazebo',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/refresher-gazebo', '855'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/refresher-ros2',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/refresher-ros2', '8eb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/exercises/advanced-exercises',
                component: ComponentCreator('/docs/ai-robot-brain/exercises/advanced-exercises', '313'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/exercises/beginner-exercises',
                component: ComponentCreator('/docs/ai-robot-brain/exercises/beginner-exercises', 'c77'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/exercises/intermediate-exercises',
                component: ComponentCreator('/docs/ai-robot-brain/exercises/intermediate-exercises', '979'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/I1-camera-depth-processing',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/I1-camera-depth-processing', '13e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/I2-tf2-coordinate-frames',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/I2-tf2-coordinate-frames', '916'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/I3-slam-toolbox',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/I3-slam-toolbox', '690'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/I4-nav2-basics',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/I4-nav2-basics', 'bcb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/',
                component: ComponentCreator('/docs/digital-twin/', 'b69'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/advanced/A1-data-synchronization',
                component: ComponentCreator('/docs/digital-twin/advanced/A1-data-synchronization', 'ac9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/advanced/A2-building-bridge',
                component: ComponentCreator('/docs/digital-twin/advanced/A2-building-bridge', 'f6c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/advanced/assets/diagrams/ai-training-architecture',
                component: ComponentCreator('/docs/digital-twin/advanced/assets/diagrams/ai-training-architecture', '686'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/beginner/B1-digital-twin-concepts',
                component: ComponentCreator('/docs/digital-twin/beginner/B1-digital-twin-concepts', '99c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/beginner/B2-first-simulation',
                component: ComponentCreator('/docs/digital-twin/beginner/B2-first-simulation', '260'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/exercises/exercise-01-launch-world',
                component: ComponentCreator('/docs/digital-twin/exercises/exercise-01-launch-world', '419'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/exercises/exercise-02-create-world',
                component: ComponentCreator('/docs/digital-twin/exercises/exercise-02-create-world', '748'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/exercises/exercise-03-build-bridge',
                component: ComponentCreator('/docs/digital-twin/exercises/exercise-03-build-bridge', '6d3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/glossary',
                component: ComponentCreator('/docs/digital-twin/glossary', '3a5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/intermediate/I1-building-worlds',
                component: ComponentCreator('/docs/digital-twin/intermediate/I1-building-worlds', '7b2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/intermediate/I2-spawning-models',
                component: ComponentCreator('/docs/digital-twin/intermediate/I2-spawning-models', '8d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/introduction',
                component: ComponentCreator('/docs/digital-twin/introduction', '08e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/',
                component: ComponentCreator('/docs/ros2-nervous-system/', 'b83'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/advanced/',
                component: ComponentCreator('/docs/ros2-nervous-system/advanced/', 'e22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/advanced/a_lesson1_urdf_humanoid',
                component: ComponentCreator('/docs/ros2-nervous-system/advanced/a_lesson1_urdf_humanoid', 'cb7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/advanced/a_lesson2_advanced_patterns',
                component: ComponentCreator('/docs/ros2-nervous-system/advanced/a_lesson2_advanced_patterns', '9e7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/advanced/exercises/advanced-exercises',
                component: ComponentCreator('/docs/ros2-nervous-system/advanced/exercises/advanced-exercises', '61c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/ai-prompts/advanced-prompts',
                component: ComponentCreator('/docs/ros2-nervous-system/ai-prompts/advanced-prompts', '0aa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/ai-prompts/beginner_ai_prompts',
                component: ComponentCreator('/docs/ros2-nervous-system/ai-prompts/beginner_ai_prompts', '3c6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/ai-prompts/intermediate_ai_prompts',
                component: ComponentCreator('/docs/ros2-nervous-system/ai-prompts/intermediate_ai_prompts', 'e03'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/beginner/',
                component: ComponentCreator('/docs/ros2-nervous-system/beginner/', '1da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/beginner/b_lesson1_intro',
                component: ComponentCreator('/docs/ros2-nervous-system/beginner/b_lesson1_intro', 'f94'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/beginner/b_lesson2_sensors',
                component: ComponentCreator('/docs/ros2-nervous-system/beginner/b_lesson2_sensors', '324'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/beginner/exercises/beginner_exercises',
                component: ComponentCreator('/docs/ros2-nervous-system/beginner/exercises/beginner_exercises', '057'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/chapter_1_glossary',
                component: ComponentCreator('/docs/ros2-nervous-system/chapter_1_glossary', '78f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/chapter_1_introduction',
                component: ComponentCreator('/docs/ros2-nervous-system/chapter_1_introduction', '6cc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/chapter_1_summary',
                component: ComponentCreator('/docs/ros2-nervous-system/chapter_1_summary', 'e4b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/intermediate/',
                component: ComponentCreator('/docs/ros2-nervous-system/intermediate/', '65a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/intermediate/exercises/intermediate_exercises',
                component: ComponentCreator('/docs/ros2-nervous-system/intermediate/exercises/intermediate_exercises', '975'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/intermediate/i_lesson1_nodes_topics',
                component: ComponentCreator('/docs/ros2-nervous-system/intermediate/i_lesson1_nodes_topics', 'd1a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/intermediate/i_lesson2_python_ros',
                component: ComponentCreator('/docs/ros2-nervous-system/intermediate/i_lesson2_python_ros', '1b6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/',
                component: ComponentCreator('/docs/workflow-orchestration/', 'db3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/beginner/b1_pipelines_flows_triggers',
                component: ComponentCreator('/docs/workflow-orchestration/beginner/b1_pipelines_flows_triggers', '405'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/beginner/b2_state_machines_concepts',
                component: ComponentCreator('/docs/workflow-orchestration/beginner/b2_state_machines_concepts', '865'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/glossary',
                component: ComponentCreator('/docs/workflow-orchestration/glossary', '738'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/introduction',
                component: ComponentCreator('/docs/workflow-orchestration/introduction', '7df'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '738'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
