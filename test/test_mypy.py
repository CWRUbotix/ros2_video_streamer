import os

from ament_mypy.main import main
import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy() -> None:
    file_path = __file__.replace(f'{__name__}.py', '')
    config_file = os.path.join(
        file_path,
        '..',
        '..',
        '..',
        '..',
        'pyproject.toml',
    )
    if os.path.exists(config_file):
        error_code = main(argv=['--config', config_file])
    else:
        error_code = main()
    assert error_code == 0, 'Found code style errors / warnings'
