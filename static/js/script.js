const socket = io();
const ros = new ROSLIB.Ros({url: 'ws://localhost:9091'});

// Управление
function startMission() {
    socket.emit('start_mission');
    addLog('Mission started');
}

function emergencyReturn() {
    if(confirm('Confirm emergency return to home position?')) {
        socket.emit('emergency_home');
        addLog('Emergency return initiated');
    }
}

// Логирование
const colors = {
    'INFO': '#00ff00',
    'WARN': '#ffff00',
    'ERROR': '#ff0000',
    'CRITICAL': '#ff00ff'
};

function addLog(message) {
    const logContainer = document.getElementById('logContainer');
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    
    const parts = message.split(']');
    const prefix = parts[0].replace('[', '');
    const content = parts.slice(1).join(']').trim();
    
    entry.innerHTML = `
        <span style="color: ${colors[prefix] || '#ffffff'}">[${prefix}]</span>
        ${content}
    `;
    
    logContainer.appendChild(entry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

// Обработчики событий
socket.on('log', data => addLog(data.data));
socket.on('trajectory', data => console.log('Trajectory updated'));
ros.on('connection', () => addLog('ROS connected'));
ros.on('error', error => addLog(`ROS error: ${error}`));
ros.on('close', () => addLog('ROS connection closed'));