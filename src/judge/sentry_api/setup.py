from setuptools import find_packages, setup

package_name = 'sentry_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm',
    maintainer_email='2949542342@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        [
            'nav_to_pose=sentry_api.nav_to_pose:main',
            'judge=sentry_api.judge:main',
            'publish=sentry_api.publish:main',
            'judge2=sentry_api.judge2:main',
            'todo2=sentry_api.todo2:main',
            'todo=sentry_api.todo:main',
            'through=sentry_api.through:main',
            'pose_state_node=sentry_api.pose_state_node:main',
            'pose_state_timer=sentry_api.pose_state_timer:main',
            'judge_tuoluo_patrol_gothroughpose.py=sentry_api.judge_tuoluo_patrol_gothroughpose:main',
            'judge_tuoluo_patrol.py=sentry_api.judge_tuoluo_patrol:main',
        ],
    },
)
