
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

# krabby_description
![Eurobot2020 simulation running][product-screenshot]
This project contains the urdf files describing the krabby robot, a robot participating to [Eurobot 2020](https://www.eurobot.org/eurobot/eurobot-2020) (Sail the world) robotics competition. The robot is actually two robots:
### Campi Robot
A camera mounted over the arena with a wide field of view

### Krabby Robot
A mobile base with two wheels, a lidar and a camera

## Prerequisites
This project uses [ROS](https://www.ros.org/), [Gazebo](http://gazebosim.org/) and have a dependency on [eurobot2020](https://github.com/Scout22/eurobot2020_gazebo). It was tested with ROS Melodic and Gazebo v9.15 on Linux, but should work with more recent versions. 
* [How to install ROS](http://wiki.ros.org/melodic/Installation) 
* [How to install Gazebo](http://gazebosim.org/tutorials?cat=install)


## Installation

Clone the project into your ROS workspace, then run ```catkin build```

## To view the urdf in Rviz2

```shell
ros2 launch krabby_description display.launch.xml
```
```
ros2 launch 
```

## Usage
To start the simulation with default parameter

```bash
roslaunch krabby_description krabby_simulation.launch
```


## Roadmap
See the open issues for a list of proposed features (and known issues).

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
Distributed under the [MIT](https://choosealicense.com/licenses/mit/) License. See `LICENSE` for more information.

## Contact
[Yanis Mazouz](ymazouz.com)

[contributors-shield]: https://img.shields.io/github/contributors/scout22/krabby_description?style=flat-square
[contributors-url]: https://github.com/scout22/krabby_description/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/scout22/krabby_description?style=social
[forks-url]: https://github.com/scout22/krabby_description/network/members
[stars-shield]: https://img.shields.io/github/stars/scout22/krabby_description?style=flat-square
[stars-url]: https://github.com/scout22/krabby_description/stargazers
[issues-shield]: https://img.shields.io/github/issues/scout22/krabby_description?style=flat-square
[issues-url]: https://github.com/scout22/krabby_description/issues
[license-shield]: https://img.shields.io/github/license/scout22/krabby_description?style=flat-square
[license-url]: https://github.com/scout22/krabby_description/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/yanis-mazouz
[product-screenshot]: images/screenshot.png
