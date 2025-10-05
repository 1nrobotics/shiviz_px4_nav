# Contributing to Shiviz PX4 Navigation

Thank you for your interest in contributing to this project!

## Code Style Guidelines

This project follows the Google coding style guides:

### C++ Code Style
- Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- Use `.clang-format` configuration provided in the repository
- Format code before committing: `clang-format -i src/*.cpp include/**/*.hpp`
- Use 2 spaces for indentation
- Maximum line length: 100 characters
- Use `snake_case` for variables and functions, `PascalCase` for classes
- Add comments for non-trivial logic

### Python Code Style
- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/) and [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
- Use docstrings (Google style) for all functions and classes
- Format code with `black` or similar formatter
- Use 4 spaces for indentation (Python default)
- Maximum line length: 100 characters
- Use type hints where appropriate
- Check code with: `flake8 src/*.py`

### Shell Scripts
- Use shellcheck for validation: `shellcheck docker/*.sh`
- Add shebang line: `#!/bin/bash`
- Use proper error handling with `set -e`
- Add comments for complex logic

### Documentation
- Keep README.md up-to-date with new features
- Add docstrings/comments for new functions
- Update configuration examples when adding parameters
- Document any breaking changes

## Development Workflow

1. **Fork and Clone**
   ```bash
   git clone --recursive https://github.com/YOUR_USERNAME/shiviz_px4_nav.git
   cd shiviz_px4_nav
   ```

2. **Create a Branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make Changes**
   - Write clean, documented code
   - Follow the style guidelines above
   - Test your changes locally

4. **Format and Lint**
   ```bash
   # C++ formatting
   clang-format -i src/*.cpp include/**/*.hpp
   
   # Python linting
   flake8 src/*.py
   
   # Shell script checking
   shellcheck docker/*.sh
   ```

5. **Test**
   - Build the code: `catkin build` or use Docker
   - Run in simulation first
   - Test on hardware if applicable
   - Verify existing functionality still works

6. **Commit**
   ```bash
   git add .
   git commit -m "Brief description of changes"
   ```
   
   Use clear, descriptive commit messages:
   - Start with a verb (Add, Fix, Update, Refactor, etc.)
   - Keep first line under 50 characters
   - Add detailed description if needed

7. **Push and Create PR**
   ```bash
   git push origin feature/your-feature-name
   ```
   Then create a Pull Request on GitHub

## Pull Request Guidelines

- Provide a clear description of what the PR does
- Reference any related issues
- Include test results or screenshots if applicable
- Ensure CI checks pass
- Be responsive to review feedback

## Project Structure

```
shiviz_px4_nav/
├── config/          # Configuration files (waypoints, parameters)
├── docker/          # Docker deployment scripts
├── include/         # C++ header files
├── launch/          # ROS launch files
├── scripts/         # Utility scripts (not part of main package)
├── src/             # Source code (C++ and Python)
│   ├── main.cpp              # Main entry point
│   ├── waypoint_navigator.cpp # Navigation logic
│   └── odom.py               # eCAL to MAVROS bridge
└── README.md        # Main documentation
```

## Reporting Issues

When reporting bugs or requesting features:

1. Check if the issue already exists
2. Provide detailed description
3. Include steps to reproduce (for bugs)
4. Specify environment (OS, ROS version, hardware)
5. Add relevant logs or error messages
6. Include configuration files if relevant

## Questions?

For questions or discussions, open an issue on GitHub or contact the maintainer.

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (TODO: add license).
