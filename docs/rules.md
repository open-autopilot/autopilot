# Developer Procedures
## Coding Style Guide
This code style guide covers conventions for naming and formatting in both C++ and Python. It includes guidelines, like naming conventions for classes, variables and functions. The guide also covers, line length limitations, and the placement of braces. These guidelines aim to improve code consistency, readability, and maintenance across both C++ and Python projects.

### Naming Conventions
- **Classes:**
```cpp
// C++ Example
class MyClass {
public:
    // class members
};
```
```python
# Python Example
class MyClass:
    def __init__(self):
        # constructor body
        pass
```

- **Variables/Functions:**
```cpp
// C++ Example
int myFunction(int input) {
    // function body
}
```
```python
# Python Example
def my_function(input):
    # function body
    pass
```

- **Constants:** 
```cpp
// C++ Example
static const int MAX_VALUE = 100;
```

### Code Formatting
- **Indentation:** Use one tab for indentation.
- **Line Length:** Limit lines length for readabilty.
- **Brace Placement:** Place opening braces on the same line.
```cpp
// C++ Example
if (condition) {
    // code here
}
```

## Testing Procedures
Our ROS 2 code testing strategy seamlessly blends code compilation and integration testing, creating a unified process within the ROS 2 ecosystem:
### Code Compilation and Integration Testing
Initiating our process, we compile the ROS 2 codebase using tools like colcon. Simultaneously, integration testing within the ROS 2 ecosystem ensures the smooth collaboration of diverse modules. This streamlined approach accelerates the testing pipeline, allowing for efficient identification of issues related to both code compilation and module interactions.

### Launch File Validation
Thoroughly evaluating ROS 2 launch files remains a priority in our testing strategy. This step, now seamlessly integrated into our unified approach, confirms proper system configuration, reducing potential issues during runtime.

### Peer Collaboration and Code Review
Prior to merging, collaborative peer reviews using Git and GitHub enhance code quality without mandating unit tests, fostering knowledge sharing among developers.

### Simulation and Real-World Validation
Our testing strategy integrates both Gazebo simulations and real-world scenario validations:

-  Gazebo Simulation Testing: Harnessing Gazebo's power, we simulate intricate robotic scenarios, offering a comprehensive testing environment without mandating unit tests.

- Real-World Testing against Specifications: Ensuring practical compliance, this step validates that our ROS 2 applications meet specific requirements outlined in specifications.

By merging the initial code compilation and integration testing phases, we create a unified foundation for our ROS 2 testing strategy. This streamlined process enhances efficiency, allowing for quicker identification and resolution of potential issues.

### Checking Specification
To prevent scope creep, checking againtst the specifications set out in the beginnen of te project will help to maintain quality. By for example when almost done with our delivarables checking which of the MOSCOW-Requirments have been implemented. 

