from glob import glob

from setuptools import setup

package_name = 'exam_proctor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaurya',
    maintainer_email='shauryajha235@yahoo.com',
    description='Gazebo simulation of a mobile exam-proctoring robot patrolling a classroom.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_node = exam_proctor_sim.patrol_node:main',
            'generate_world = exam_proctor_sim.generate_world:main',
            'student_behaviors = exam_proctor_sim.student_behaviors:main',
        ],
    },
)
