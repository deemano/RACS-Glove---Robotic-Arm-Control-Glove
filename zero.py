import subprocess

# SSH connection details
hostname = '10.0.243.26'
username = 'deeman'
password = '12345678'

def ssh_connect(hostname, username, password):
    try:
        # Create the SSH command
        ssh_command = f'ssh {username}@{hostname}'

        # Start the SSH session using subprocess
        subprocess.run(['cmd', '/c', ssh_command])
    except Exception as e:
        print(f"Failed to connect to {hostname}: {e}")

if __name__ == "__main__":
    ssh_connect(hostname, username, password)
