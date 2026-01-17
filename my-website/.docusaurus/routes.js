import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5fa'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '11f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '5d7'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '591'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '78b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '2d7'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'c1c'),
    exact: true
  },
  {
    path: '/auth',
    component: ComponentCreator('/auth', '13a'),
    exact: true
  },
  {
    path: '/login',
    component: ComponentCreator('/login', '6eb'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', '1ab'),
    exact: true
  },
  {
    path: '/what-you-learn',
    component: ComponentCreator('/what-you-learn', '20a'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '041'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '373'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'f9e'),
            routes: [
              {
                path: '/docs/ai-robot-brain/',
                component: ComponentCreator('/docs/ai-robot-brain/', '3ee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/', '441'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/a_lesson1_costmap',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/a_lesson1_costmap', 'd36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/a_lesson2_planners',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/a_lesson2_planners', 'ab7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/a_lesson3_rl',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/a_lesson3_rl', '191'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/advanced/a_lesson4_sim_to_real',
                component: ComponentCreator('/docs/ai-robot-brain/advanced/a_lesson4_sim_to_real', 'aa3'),
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
                path: '/docs/ai-robot-brain/ai-prompts/chapter_3_advanced_prompts',
                component: ComponentCreator('/docs/ai-robot-brain/ai-prompts/chapter_3_advanced_prompts', 'd53'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/ai-prompts/chapter_3_beginner_prompts',
                component: ComponentCreator('/docs/ai-robot-brain/ai-prompts/chapter_3_beginner_prompts', 'aaf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/ai-prompts/chapter_3_intermediate_prompts',
                component: ComponentCreator('/docs/ai-robot-brain/ai-prompts/chapter_3_intermediate_prompts', 'a88'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/', '615'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/b_lesson1_perception',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/b_lesson1_perception', 'd3e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/b_lesson2_sensors',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/b_lesson2_sensors', '05a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/beginner/b_lesson3_slam_nav',
                component: ComponentCreator('/docs/ai-robot-brain/beginner/b_lesson3_slam_nav', 'e2b'),
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
                path: '/docs/ai-robot-brain/chapter_3_glossary',
                component: ComponentCreator('/docs/ai-robot-brain/chapter_3_glossary', '252'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/chapter_3_introduction',
                component: ComponentCreator('/docs/ai-robot-brain/chapter_3_introduction', '6c8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/chapter_3_summary',
                component: ComponentCreator('/docs/ai-robot-brain/chapter_3_summary', '020'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/exercises/chapter_3_advanced_exercises',
                component: ComponentCreator('/docs/ai-robot-brain/exercises/chapter_3_advanced_exercises', '98f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/exercises/chapter_3_beginner_exercises',
                component: ComponentCreator('/docs/ai-robot-brain/exercises/chapter_3_beginner_exercises', '052'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/exercises/chapter_3_intermediate_exercises',
                component: ComponentCreator('/docs/ai-robot-brain/exercises/chapter_3_intermediate_exercises', '112'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/', '314'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/i_lesson1_camera_depth',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/i_lesson1_camera_depth', '470'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/i_lesson2_tf2',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/i_lesson2_tf2', '1ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/i_lesson3_slam_toolbox',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/i_lesson3_slam_toolbox', '847'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-robot-brain/intermediate/i_lesson4_nav2',
                component: ComponentCreator('/docs/ai-robot-brain/intermediate/i_lesson4_nav2', 'd65'),
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
                path: '/docs/digital-twin/advanced/assets/diagrams/ai-training-architecture',
                component: ComponentCreator('/docs/digital-twin/advanced/assets/diagrams/ai-training-architecture', '686'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/advanced/chapter_2_lesson_a1',
                component: ComponentCreator('/docs/digital-twin/advanced/chapter_2_lesson_a1', '4f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/advanced/chapter_2_lesson_a2',
                component: ComponentCreator('/docs/digital-twin/advanced/chapter_2_lesson_a2', 'b79'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/beginner/chapter_2_lesson_b1',
                component: ComponentCreator('/docs/digital-twin/beginner/chapter_2_lesson_b1', 'c16'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/beginner/chapter_2_lesson_b2',
                component: ComponentCreator('/docs/digital-twin/beginner/chapter_2_lesson_b2', 'c49'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/chapter_2_glossary',
                component: ComponentCreator('/docs/digital-twin/chapter_2_glossary', '429'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/chapter_2_introduction',
                component: ComponentCreator('/docs/digital-twin/chapter_2_introduction', 'bc9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/exercises/chapter_2_exercise_1',
                component: ComponentCreator('/docs/digital-twin/exercises/chapter_2_exercise_1', 'e3a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/exercises/chapter_2_exercise_2',
                component: ComponentCreator('/docs/digital-twin/exercises/chapter_2_exercise_2', '4ed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/exercises/chapter_2_exercise_3',
                component: ComponentCreator('/docs/digital-twin/exercises/chapter_2_exercise_3', '955'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/intermediate/chapter_2_lesson_i1',
                component: ComponentCreator('/docs/digital-twin/intermediate/chapter_2_lesson_i1', 'cb9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/intermediate/chapter_2_lesson_i2',
                component: ComponentCreator('/docs/digital-twin/intermediate/chapter_2_lesson_i2', '78c'),
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
                path: '/docs/ros2-nervous-system/advanced/exercises/advanced_exercises',
                component: ComponentCreator('/docs/ros2-nervous-system/advanced/exercises/advanced_exercises', '337'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-nervous-system/ai-prompts/advanced_ai_prompts',
                component: ComponentCreator('/docs/ros2-nervous-system/ai-prompts/advanced_ai_prompts', '86f'),
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
                path: '/docs/workflow-orchestration/advanced/',
                component: ComponentCreator('/docs/workflow-orchestration/advanced/', '343'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/advanced/a1_watchdogs_health_monitoring',
                component: ComponentCreator('/docs/workflow-orchestration/advanced/a1_watchdogs_health_monitoring', 'df8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/advanced/a2_supervisor_recovery',
                component: ComponentCreator('/docs/workflow-orchestration/advanced/a2_supervisor_recovery', '539'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/advanced/a3_continuous_operation',
                component: ComponentCreator('/docs/workflow-orchestration/advanced/a3_continuous_operation', 'f6a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/advanced/exercises/advanced_exercises_workflow',
                component: ComponentCreator('/docs/workflow-orchestration/advanced/exercises/advanced_exercises_workflow', 'c22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/ai-prompts/advanced_prompts_workflow',
                component: ComponentCreator('/docs/workflow-orchestration/ai-prompts/advanced_prompts_workflow', 'ff2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/ai-prompts/beginner_prompts_workflow',
                component: ComponentCreator('/docs/workflow-orchestration/ai-prompts/beginner_prompts_workflow', 'e51'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/ai-prompts/intermediate_prompts_workflow',
                component: ComponentCreator('/docs/workflow-orchestration/ai-prompts/intermediate_prompts_workflow', '7c1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/beginner/',
                component: ComponentCreator('/docs/workflow-orchestration/beginner/', 'f68'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/beginner/b1_pipelines_flows_triggers',
                component: ComponentCreator('/docs/workflow-orchestration/beginner/b1_pipelines_flows_triggers', '412'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/beginner/b2_state_machines_concepts',
                component: ComponentCreator('/docs/workflow-orchestration/beginner/b2_state_machines_concepts', 'fd7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/beginner/exercises/beginner_exercises_workflow',
                component: ComponentCreator('/docs/workflow-orchestration/beginner/exercises/beginner_exercises_workflow', '9f7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/chapter_4_glossary',
                component: ComponentCreator('/docs/workflow-orchestration/chapter_4_glossary', 'f64'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/chapter_4_introduction',
                component: ComponentCreator('/docs/workflow-orchestration/chapter_4_introduction', '778'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/chapter_4_summary',
                component: ComponentCreator('/docs/workflow-orchestration/chapter_4_summary', '123'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/intermediate/',
                component: ComponentCreator('/docs/workflow-orchestration/intermediate/', '69f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/intermediate/exercises/intermediate_exercises_workflow',
                component: ComponentCreator('/docs/workflow-orchestration/intermediate/exercises/intermediate_exercises_workflow', '458'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/intermediate/i1_state_machines_ros2',
                component: ComponentCreator('/docs/workflow-orchestration/intermediate/i1_state_machines_ros2', 'b29'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/intermediate/i2_multi_node_pipelines',
                component: ComponentCreator('/docs/workflow-orchestration/intermediate/i2_multi_node_pipelines', '34e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/workflow-orchestration/intermediate/i3_inter_node_communication',
                component: ComponentCreator('/docs/workflow-orchestration/intermediate/i3_inter_node_communication', 'ce1'),
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
    component: ComponentCreator('/', '2b7'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
