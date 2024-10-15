# Multi Robot Simulation (MuRoSim)

Multi-robot navigation is a challenging task in which multiple robots must be coordinated simultaneously within dynamic environments. 
We apply deep reinforcement learning (DRL) to learn a decentralized end-to-end policy which maps raw sensor data to the command velocities of the agent. 
In order to enable the policy to generalize, the training is performed in different environments and scenarios. 
The learned policy is tested and evaluated in common multi-robot scenarios like switching a place, an intersection and a bottleneck situation. 
This policy allows the agent to recover from dead ends and to navigate through complex environments.

**Source code is getting prepared to be released.**


### Academic Attribution

MuRoSim is a fast and efficient multi-robot simulation for learning-based navigation and was developed at the Fraunhofer Institute for Material Flow and Logistics (IML). 
If you use it for research, please include one of the following references in any resulting publication.

- [MuRoSim – A Fast and Efficient Multi-Robot Simulation for Learning-based Navigation](https://doi.org/10.1109/ICRA57147.2024.10610375)
```plain
@INPROCEEDINGS{JestIcra2024,
  author={Jestel, Christian and Rösner, Karol and Dietz, Niklas and Bach, Nicolas and Eßer, Julian and Finke, Jan and Urbann, Oliver},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={MuRoSim – A Fast and Efficient Multi-Robot Simulation for Learning-based Navigation}, 
  year={2024},
  volume={},
  number={},
  pages={16881-16887},
  keywords={Training;Laser radar;Navigation;Instruction sets;Robot sensing systems;Robot learning;Sensors},
  doi={10.1109/ICRA57147.2024.10610375}}
```
