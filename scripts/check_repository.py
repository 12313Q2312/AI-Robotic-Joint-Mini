"""Check tracked delivery files without running a simulator or installing tools."""
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'.git', '__pycache__', '.venv', 'node_modules', 'build', 'install', 'log'}
BANNED = {'.pem', '.key', '.exe', '.dll', '.so', '.o', '.obj', '.pyc', '.zip'}
REQUIRED = ('README.md', 'PROJECT_STATUS.md', 'CONTRIBUTING.md', 'SECURITY.md',
            'LICENSE', 'THIRD_PARTY_NOTICES.md', 'docs/REPRODUCIBILITY.md',
            'docs/LIMITATIONS.md', '.github/CODEOWNERS', '.github/workflows/ci.yml')
SECRET_PATTERNS = (
    re.compile(r'\bgh[pousr]_[A-Za-z0-9]{30,}\b'),
    re.compile(r'\bgithub_pat_[A-Za-z0-9_]{40,}\b'),
    re.compile(r'\bsk-(?:proj-)?[A-Za-z0-9_-]{32,}\b'),
    re.compile(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'),
)

def files():
    # Tracked paths prevent ignored local runs/builds from breaking CI hygiene.
    if (ROOT / '.git').exists():
        result = subprocess.run(['git', 'ls-files', '-z'], cwd=ROOT,
                                capture_output=True, check=True)
        return [ROOT / p.decode('utf-8') for p in result.stdout.split(b'\0') if p]
    return [p for p in ROOT.rglob('*') if p.is_file()
            and not set(p.relative_to(ROOT).parts) & SKIP]

def main():
    errors = []
    for name in REQUIRED:
        if not (ROOT / name).is_file():
            errors.append(f'Missing required file: {name}')
    paths = files()
    for p in paths:
        rel = p.relative_to(ROOT).as_posix()
        if p.suffix.lower() in BANNED or (p.name.startswith('.env') and p.name != '.env.example'):
            errors.append(f'Excluded artifact or credential filename: {rel}')
        if p.stat().st_size > 5 * 1024 * 1024:
            errors.append(f'File exceeds 5 MiB delivery budget: {rel}')
        raw = p.read_bytes()
        if b'\x00' in raw:
            continue
        try:
            text = raw.decode('utf-8-sig')
        except UnicodeDecodeError:
            continue
        if re.search(r'(?m)^(?:<<<<<<< |=======\s*$|>>>>>>> )', text):
            errors.append(f'Unresolved merge marker: {rel}')
        if any(pattern.search(text) for pattern in SECRET_PATTERNS):
            errors.append(f'Potential credential: {rel} (value withheld)')
    literal = subprocess.run([sys.executable, str(ROOT / 'scripts/check_actuator_limit_literal.py')], cwd=ROOT, check=False)
    if literal.returncode != 0:
        errors.append('Duplicated actuator torque-limit literal')
    try:
        ET.parse(ROOT / 'ros2_ws/src/ai_robotic_joint_ros2/package.xml')
    except (ET.ParseError, OSError) as exc:
        errors.append(f'Invalid ROS package metadata: {exc}')
    for name in ('README.md', 'CONTRIBUTING.md', 'docs/REPRODUCIBILITY.md', 'docs/LIMITATIONS.md'):
        p = ROOT / name
        if not p.exists():
            continue
        for link in re.findall(r'\]\(([^)]+)\)', p.read_text(encoding='utf-8')):
            if '://' in link or link.startswith('#'):
                continue
            target = link.split('#', 1)[0]
            if target and not (p.parent / target).exists():
                errors.append(f'Broken navigation link in {name}: {link}')
    for error in errors:
        print(error)
    if errors:
        print(f'FAIL: {len(errors)} repository checks')
        return 1
    print(f'PASS: {len(paths)} files; required docs, navigation, metadata and credential-pattern checks')
    return 0

if __name__ == '__main__':
    sys.exit(main())
