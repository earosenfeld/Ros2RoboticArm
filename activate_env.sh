#!/bin/bash

# Add local bin to PATH for pip and virtualenv
export PATH="$HOME/.local/bin:$PATH"

# Activate the virtual environment
source venv/bin/activate

echo "Virtual environment activated!"
echo "Python version: $(python --version)"
echo "To deactivate, run: deactivate" 