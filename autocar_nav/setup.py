entry_points={
    'console_scripts': [
        'tracker = autocar_nav.nodes.tracker:main',
        'autocar_tf_ros2 = autocar_nav.nodes.autocar_tf_ros2:main',
        'hitech_test_pub = autocar_nav.nodes.hitech_test_pub:main',
        'mpc_tracker = autocar_nav.nodes.mpc_tracker:main',
        'global_planner = autocar_nav.nodes.global_planner:main'
    ],
}, 