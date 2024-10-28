# romea_ros2_teleop #

This project contains several ROS2 packages for working with GPS receivers, both in real-world applications and simulations. For more detailed information, please refer to the README files of each individual package.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-ros2-gps/refs/heads/main/romea_gps_public.repos
5. vcs import src < romea_gps_public.repos
6. colcon build
7. see bringup package to launch gps

## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea ROS2 GPS stack was developed by **Jean Laneurit** in the context of various research projects carried out at INRAE.

## **Contact**

If you have any questions or comments about Romea ROS2 GPS stack, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 