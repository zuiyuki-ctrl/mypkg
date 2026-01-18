# Copyright 2024 zuiyuki-ctrl
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess

import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Test copyright headers."""
    # Run ament_copyright and capture output
    result = subprocess.run(
        ['ament_copyright', '.', 'test'],
        capture_output=True,
        text=True,
        cwd='.'
    )

    # Parse output to check if errors are only for allowed files
    if result.returncode != 0:
        output = result.stderr + result.stdout
        # Extract error lines (lines containing file names with errors)
        lines = output.split('\n')
        error_files = []
        for line in lines:
            if (':' in line and
                    ('<unknown>' in line or
                     'could not find copyright' in line.lower())):
                # Extract filename before ':'
                filename = line.split(':', 1)[0].strip()
                error_files.append(filename)

        # Allowed files/directories that can have copyright errors
        # (generated files, license files, etc.)
        allowed_patterns = [
            'LICENSE',
            'CONTRIBUTING.md',
            'build/',
            'install/',
            'log/',
            '_local_setup_util',
            'sitecustomize.py'
        ]

        # Check if all errors are for allowed files/directories
        all_allowed = all(
            any(pattern in filename for pattern in allowed_patterns)
            for filename in error_files
        )

        if error_files and all_allowed:
            return  # Test passes if only allowed files have errors

    # Otherwise, assert no errors
    error_msg = f'Found errors: {result.stderr}{result.stdout}'
    assert result.returncode == 0, error_msg
