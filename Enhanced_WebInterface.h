// Enhanced_WebInterface.h
#ifndef ENHANCED_WEBINTERFACE_H
#define ENHANCED_WEBINTERFACE_H

#include "Config.h"
#include <ESPAsyncWebServer.h>

const char MULTICROP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>🌾 Multi-Crop Precision Seed Drill</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body { 
            font-family: 'Segoe UI', Arial, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); 
            color: #333; 
            min-height: 100vh;
        }
        
        .container { 
            max-width: 1400px; 
            margin: 0 auto; 
            padding: 20px;
        }
        
        .header { 
            background: rgba(255,255,255,0.95); 
            text-align: center; 
            margin-bottom: 20px; 
            padding: 20px; 
            border-radius: 15px; 
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        }
        
        .flag-colors {
            background: linear-gradient(to right, #ff9933 33%, #ffffff 33%, #ffffff 66%, #138808 66%);
            height: 5px;
            margin: 10px 0;
        }
        
        .tabs {
            display: flex;
            background: rgba(255,255,255,0.95);
            border-radius: 15px 15px 0 0;
            overflow: hidden;
            box-shadow: 0 4px 15px rgba(0,0,0,0.2);
        }
        
        .tab {
            flex: 1;
            padding: 15px 20px;
            background: #f8f9fa;
            border: none;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.3s ease;
            border-right: 1px solid #ddd;
        }
        
        .tab:last-child { border-right: none; }
        
        .tab.active {
            background: #007bff;
            color: white;
            transform: translateY(-2px);
        }
        
        .tab:hover:not(.active) {
            background: #e9ecef;
        }
        
        .tab-content {
            display: none;
            background: rgba(255,255,255,0.95);
            border-radius: 0 0 15px 15px;
            padding: 25px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        }
        
        .tab-content.active {
            display: block;
        }
        
        /* Control Panel Styles */
        .control-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .control-card {
            background: linear-gradient(145deg, #f8f9fa, #e9ecef);
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.1);
        }
        
        .control-card h3 {
            color: #007bff;
            margin-bottom: 15px;
            font-size: 1.2em;
        }
        
        .setting-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin: 10px 0;
            padding: 8px;
            background: white;
            border-radius: 6px;
        }
        
        .setting-label {
            font-weight: 600;
            min-width: 120px;
        }
        
        .setting-input, select {
            padding: 8px 12px;
            border: 2px solid #ddd;
            border-radius: 6px;
            font-size: 1em;
            min-width: 120px;
        }
        
        .setting-input:focus, select:focus {
            border-color: #007bff;
            outline: none;
        }
        
        /* Status Display */
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }
        
        .status-card {
            background: white;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 4px solid #28a745;
        }
        
        .status-label {
            font-size: 0.9em;
            color: #666;
            margin-bottom: 8px;
        }
        
        .status-value {
            font-size: 1.8em;
            font-weight: bold;
            color: #28a745;
        }
        
        .status-unit {
            font-size: 0.8em;
            color: #999;
        }
        
        /* Crop Selection */
        .crop-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }
        
        .crop-card {
            background: linear-gradient(145deg, #f8f9fa, #e9ecef);
            padding: 20px;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s ease;
            border: 3px solid transparent;
            text-align: center;
        }
        
        .crop-card:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 25px rgba(0,0,0,0.2);
            border-color: #007bff;
        }
        
        .crop-card.selected {
            border-color: #28a745;
            background: linear-gradient(145deg, #d4edda, #c3e6cb);
        }
        
        .crop-icon { font-size: 2.5em; margin-bottom: 10px; }
        .crop-name { font-size: 1.2em; font-weight: bold; color: #007bff; }
        .crop-details { font-size: 0.9em; color: #666; margin: 5px 0; }
        
        /* Data Logging */
        .log-container {
            background: #f8f9fa;
            border-radius: 10px;
            padding: 20px;
            margin: 20px 0;
        }
        
        .log-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 15px;
        }
        
        .log-data {
            background: white;
            border-radius: 8px;
            padding: 15px;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            max-height: 300px;
            overflow-y: auto;
            border: 1px solid #ddd;
        }
        
        .log-entry {
            padding: 5px 0;
            border-bottom: 1px solid #eee;
        }
        
        .log-entry:last-child {
            border-bottom: none;
        }
        
        /* Buttons */
        .btn {
            background: linear-gradient(145deg, #007bff, #0056b3);
            color: white;
            border: none;
            padding: 12px 20px;
            margin: 5px;
            border-radius: 6px;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.3s ease;
        }
        
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 15px rgba(0,123,255,0.4);
        }
        
        .btn.success { background: linear-gradient(145deg, #28a745, #1e7e34); }
        .btn.warning { background: linear-gradient(145deg, #ffc107, #e0a800); }
        .btn.danger { background: linear-gradient(145deg, #dc3545, #c82333); }
        .btn.secondary { background: linear-gradient(145deg, #6c757d, #545b62); }
        
        .btn:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            transform: none;
        }
        
        /* Test Mode Panel */
        .test-panel {
            background: linear-gradient(145deg, #fff3cd, #ffeaa7);
            border: 2px solid #ffc107;
            border-radius: 10px;
            padding: 20px;
            margin: 20px 0;
        }
        
        .test-panel.active {
            background: linear-gradient(145deg, #d1ecf1, #bee5eb);
            border-color: #17a2b8;
        }
        
        /* Responsive Design */
        @media (max-width: 768px) {
            .tabs { flex-direction: column; }
            .control-grid { grid-template-columns: 1fr; }
            .status-grid { grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); }
            .crop-grid { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
<div class="container">
    <!-- Header -->
    <div class="header">
        <div class="flag-colors"></div>
        <h1>🌾 Multi-Crop Precision Seed Drill</h1>
        <p>🇮🇳 Advanced Agricultural Solution for Indian Farming</p>
        <div class="flag-colors"></div>
    </div>
    
    <!-- Tab Navigation -->
    <div class="tabs">
        <button class="tab active" onclick="showTab('control')">🎮 Control Panel</button>
        <button class="tab" onclick="showTab('crops')">🌱 Crop Selection</button>
        <button class="tab" onclick="showTab('test')">🧪 Test Mode</button>
        <button class="tab" onclick="showTab('data')">📊 Data Logging</button>
        <button class="tab" onclick="showTab('settings')">⚙️ Settings</button>
    </div>
    
    <!-- Control Panel Tab -->
    <div id="control-tab" class="tab-content active">
        <div class="control-grid">
            <!-- System Status -->
            <div class="control-card">
                <h3>📊 System Status</h3>
                <div class="setting-row">
                    <span class="setting-label">System:</span>
                    <span id="system-status" class="status-value">READY</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">GPS:</span>
                    <span id="gps-status">Searching...</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Motor:</span>
                    <span id="motor-status">STOPPED</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Current Crop:</span>
                    <span id="current-crop">Not Selected</span>
                </div>
            </div>
            
            <!-- Real-time Values -->
            <div class="control-card">
                <h3>📈 Real-time Data</h3>
                <div class="setting-row">
                    <span class="setting-label">Speed:</span>
                    <span id="current-speed">0.0 km/h</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">RPM:</span>
                    <span id="current-rpm">0</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">PWM:</span>
                    <span id="current-pwm">0</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Satellites:</span>
                    <span id="satellite-count">0</span>
                </div>
            </div>
            
            <!-- Field Configuration -->
            <div class="control-card">
                <h3>🌾 Field Settings</h3>
                <div class="setting-row">
                    <span class="setting-label">Soil Type:</span>
                    <select id="soil-type">
                        <option value="loamy">Loamy (दोमट)</option>
                        <option value="clay">Clay (चिकनी)</option>
                        <option value="sandy">Sandy (बलुई)</option>
                        <option value="black-cotton">Black Cotton (काली कपास)</option>
                        <option value="alluvial">Alluvial (जलोढ़)</option>
                        <option value="red-laterite">Red Laterite (लाल लेटराइट)</option>
                    </select>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Season:</span>
                    <select id="season">
                        <option value="kharif">Kharif - Monsoon (खरीफ)</option>
                        <option value="rabi">Rabi - Winter (रबी)</option>
                        <option value="zaid">Zaid - Summer (जायद)</option>
                    </select>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Working Speed:</span>
                    <input type="range" id="speed-slider" min="2" max="12" step="0.5" value="5" 
                           oninput="updateSpeedDisplay()" style="flex: 1; margin: 0 10px;">
                    <span id="speed-display">5.0 km/h</span>
                </div>
            </div>
        </div>
        
        <!-- Status Cards -->
        <div class="status-grid">
            <div class="status-card">
                <div class="status-label">Target Spacing</div>
                <div class="status-value" id="calc-spacing">--</div>
                <div class="status-unit">cm</div>
            </div>
            <div class="status-card">
                <div class="status-label">Seed Rate</div>
                <div class="status-value" id="calc-rate">--</div>
                <div class="status-unit">kg/ha</div>
            </div>
            <div class="status-card">
                <div class="status-label">Expected Yield</div>
                <div class="status-value" id="calc-yield">--</div>
                <div class="status-unit">kg/ha</div>
            </div>
            <div class="status-card">
                <div class="status-label">Operating Cost</div>
                <div class="status-value" id="calc-cost">--</div>
                <div class="status-unit">₹/ha</div>
            </div>
        </div>
        
        <!-- Control Buttons -->
        <div style="text-align: center; margin: 20px 0;">
            <button class="btn success" onclick="applyConfiguration()">✅ Apply Configuration</button>
            <button class="btn" onclick="startCalibration()">⚙️ Calibrate System</button>
            <button class="btn danger" onclick="emergencyStop()">🛑 Emergency Stop</button>
        </div>
    </div>
    
    <!-- Crop Selection Tab -->
    <div id="crops-tab" class="tab-content">
        <h2>🌱 Select Crop Type</h2>
        <div class="crop-grid" id="crop-grid">
            <!-- Crops will be populated by JavaScript -->
        </div>
    </div>
    
    <!-- Test Mode Tab -->
    <div id="test-tab" class="tab-content">
        <div class="test-panel" id="test-panel">
            <h2>🧪 Test Mode</h2>
            <p>Test mode allows you to run the system at a fixed speed without GPS input.</p>
            
            <div class="control-grid" style="margin: 20px 0;">
                <div class="control-card">
                    <h3>Test Parameters</h3>
                    <div class="setting-row">
                        <span class="setting-label">Test Speed:</span>
                        <input type="number" id="test-speed" min="0.5" max="15" step="0.1" value="5.0" class="setting-input">
                        <span>km/h</span>
                    </div>
                    <div class="setting-row">
                        <span class="setting-label">Test Duration:</span>
                        <input type="number" id="test-duration" min="10" max="3600" value="60" class="setting-input">
                        <span>seconds</span>
                    </div>
                </div>
                
                <div class="control-card">
                    <h3>Test Status</h3>
                    <div class="setting-row">
                        <span class="setting-label">Mode:</span>
                        <span id="test-mode-status">DISABLED</span>
                    </div>
                    <div class="setting-row">
                        <span class="setting-label">Remaining:</span>
                        <span id="test-time-remaining">--</span>
                    </div>
                    <div class="setting-row">
                        <span class="setting-label">Test RPM:</span>
                        <span id="test-rpm">--</span>
                    </div>
                </div>
            </div>
            
            <div style="text-align: center;">
                <button class="btn success" id="start-test-btn" onclick="startTestMode()">▶️ Start Test</button>
                <button class="btn warning" id="stop-test-btn" onclick="stopTestMode()" disabled>⏹️ Stop Test</button>
            </div>
        </div>
    </div>
    
    <!-- Data Logging Tab -->
    <div id="data-tab" class="tab-content">
        <div class="log-container">
            <div class="log-header">
                <h2>📊 Data Logging</h2>
                <div>
                    <button class="btn secondary" onclick="clearLog()">🗑️ Clear Log</button>
                    <button class="btn success" onclick="downloadLog()">⬇️ Download CSV</button>
                    <button class="btn" onclick="toggleLogging()" id="logging-btn">▶️ Start Logging</button>
                </div>
            </div>
            
            <div class="control-grid" style="margin-bottom: 20px;">
                <div class="control-card">
                    <h3>Logging Settings</h3>
                    <div class="setting-row">
                        <span class="setting-label">Log Interval:</span>
                        <select id="log-interval">
                            <option value="1000">1 second</option>
                            <option value="5000" selected>5 seconds</option>
                            <option value="10000">10 seconds</option>
                            <option value="30000">30 seconds</option>
                        </select>
                    </div>
                    <div class="setting-row">
                        <span class="setting-label">Status:</span>
                        <span id="logging-status">STOPPED</span>
                    </div>
                    <div class="setting-row">
                        <span class="setting-label">Entries:</span>
                        <span id="log-count">0</span>
                    </div>
                </div>
                
                <div class="control-card">
                    <h3>Data Format</h3>
                    <p>CSV columns: Timestamp, Mode, Speed(km/h), RPM, PWM, Satellites, HDOP, Crop</p>
                    <p>Example: 2024-01-15 10:30:25, FIELD, 4.5, 152, 95, 8, 1.2, Wheat</p>
                </div>
            </div>
            
            <div class="log-data" id="log-display">
                <div class="log-entry">Timestamp, Mode, Speed(km/h), RPM, PWM, Satellites, HDOP, Crop</div>
                <div class="log-entry">--- No data logged yet ---</div>
            </div>
        </div>
    </div>
    
    <!-- Settings Tab -->
    <div id="settings-tab" class="tab-content">
        <div class="control-grid">
            <div class="control-card">
                <h3>⚙️ System Settings</h3>
                <div class="setting-row">
                    <span class="setting-label">Auto-refresh:</span>
                    <input type="checkbox" id="auto-refresh" checked>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Refresh Rate:</span>
                    <select id="refresh-rate">
                        <option value="1000">1 second</option>
                        <option value="2000" selected>2 seconds</option>
                        <option value="5000">5 seconds</option>
                    </select>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Sound Alerts:</span>
                    <input type="checkbox" id="sound-alerts" checked>
                </div>
            </div>
            
            <div class="control-card">
                <h3>🔧 Maintenance</h3>
                <button class="btn warning" onclick="performDiagnostics()">🔍 Run Diagnostics</button>
                <button class="btn secondary" onclick="exportSettings()">📤 Export Settings</button>
                <button class="btn danger" onclick="factoryReset()">🏭 Factory Reset</button>
            </div>
            
            <div class="control-card">
                <h3>📱 Connection</h3>
                <div class="setting-row">
                    <span class="setting-label">ESP32 IP:</span>
                    <span id="esp-ip">192.168.4.1</span>
                </div>
                <div class="setting-row">
                    <span class="setting-label">Connection:</span>
                    <span id="connection-status">Connected</span>
                </div>
                <button class="btn" onclick="reconnect()">🔄 Reconnect</button>
            </div>
        </div>
    </div>
</div>

<script>
// Global variables
let selectedCrop = null;
let testModeActive = false;
let loggingActive = false;
let logData = [];
let testTimer = null;
let refreshTimer = null;

// Crop database
const crops = [
    { id: 'wheat', icon: '🌾', name: 'Wheat', hindi: 'गेहूं', spacing: 15, rate: 120, yield: 4500, season: 'Rabi' },
    { id: 'soybean', icon: '🫘', name: 'Soybean', hindi: 'सोयाबीन', spacing: 20, rate: 80, yield: 2500, season: 'Kharif' },
    { id: 'cotton', icon: '🌸', name: 'Cotton', hindi: 'कपास', spacing: 30, rate: 25, yield: 1800, season: 'Kharif' },
    { id: 'groundnut', icon: '🥜', name: 'Groundnut', hindi: 'मूंगफली', spacing: 18, rate: 100, yield: 2200, season: 'Kharif' },
    { id: 'mustard', icon: '🌻', name: 'Mustard', hindi: 'सरसों', spacing: 12, rate: 5, yield: 1500, season: 'Rabi' },
    { id: 'sunflower', icon: '🌻', name: 'Sunflower', hindi: 'सूरजमुखी', spacing: 25, rate: 10, yield: 2000, season: 'Kharif/Rabi' },
    { id: 'maize', icon: '🌽', name: 'Maize', hindi: 'मक्का', spacing: 22, rate: 20, yield: 6000, season: 'Kharif' },
    { id: 'chickpea', icon: '🫛', name: 'Chickpea', hindi: 'चना', spacing: 16, rate: 80, yield: 2800, season: 'Rabi' }
];

// Initialize page
document.addEventListener('DOMContentLoaded', function() {
    initializeCropGrid();
    startAutoRefresh();
    updateStatus();
});

// Tab management
function showTab(tabName) {
    // Hide all tabs
    document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
    document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
    
    // Show selected tab
    document.getElementById(tabName + '-tab').classList.add('active');
    event.target.classList.add('active');
}

// Initialize crop grid
function initializeCropGrid() {
    const grid = document.getElementById('crop-grid');
    grid.innerHTML = '';
    
    crops.forEach(crop => {
        const card = document.createElement('div');
        card.className = 'crop-card';
        card.id = `crop-${crop.id}`;
        card.onclick = () => selectCrop(crop.id);
        
        card.innerHTML = `
            <div class="crop-icon">${crop.icon}</div>
            <div class="crop-name">${crop.name}</div>
            <div class="crop-name" style="font-size: 1em; color: #666;">${crop.hindi}</div>
            <div class="crop-details">
                Spacing: ${crop.spacing}cm • ${crop.season}<br>
                Rate: ${crop.rate} kg/ha • Yield: ${crop.yield} kg/ha
            </div>
        `;
        
        grid.appendChild(card);
    });
}

// Crop selection
function selectCrop(cropId) {
    document.querySelectorAll('.crop-card').forEach(card => card.classList.remove('selected'));
    document.getElementById(`crop-${cropId}`).classList.add('selected');
    
    selectedCrop = crops.find(c => c.id === cropId);
    document.getElementById('current-crop').textContent = selectedCrop.name;
    
    updateCalculations();
}

// Update calculations
function updateCalculations() {
    if (!selectedCrop) return;
    
    document.getElementById('calc-spacing').textContent = selectedCrop.spacing;
    document.getElementById('calc-rate').textContent = selectedCrop.rate;
    document.getElementById('calc-yield').textContent = selectedCrop.yield;
    
    const speed = parseFloat(document.getElementById('speed-slider').value);
    const cost = Math.round(2000 + (speed * 100));
    document.getElementById('calc-cost').textContent = '₹' + cost;
}

// Speed slider update
function updateSpeedDisplay() {
    const speed = parseFloat(document.getElementById('speed-slider').value);
    document.getElementById('speed-display').textContent = speed.toFixed(1) + ' km/h';
    updateCalculations();
}

// Test mode functions
function startTestMode() {
    const speed = parseFloat(document.getElementById('test-speed').value);
    const duration = parseInt(document.getElementById('test-duration').value);
    
    if (!selectedCrop) {
        alert('Please select a crop first!');
        return;
    }
    
    testModeActive = true;
    document.getElementById('test-mode-status').textContent = 'ACTIVE';
    document.getElementById('test-panel').classList.add('active');
    document.getElementById('start-test-btn').disabled = true;
    document.getElementById('stop-test-btn').disabled = false;
    
    // Calculate test RPM
    const rpm = Math.round(speed * selectedCrop.spacing * 1.5);
    document.getElementById('test-rpm').textContent = rpm;
    
    // Start countdown timer
    let remaining = duration;
    testTimer = setInterval(() => {
        remaining--;
        const mins = Math.floor(remaining / 60);
        const secs = remaining % 60;
        document.getElementById('test-time-remaining').textContent = 
            `${mins}:${secs.toString().padStart(2, '0')}`;
        
        if (remaining <= 0) {
            stopTestMode();
        }
    }, 1000);
    
    // Send test mode command to ESP32
    fetch('/test_mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speed, duration: duration, crop: selectedCrop.id })
    });
    
    alert(`Test mode started: ${speed} km/h for ${duration} seconds`);
}

function stopTestMode() {
    testModeActive = false;
    document.getElementById('test-mode-status').textContent = 'DISABLED';
    document.getElementById('test-panel').classList.remove('active');
    document.getElementById('start-test-btn').disabled = false;
    document.getElementById('stop-test-btn').disabled = true;
    document.getElementById('test-time-remaining').textContent = '--';
    document.getElementById('test-rpm').textContent = '--';
    
    if (testTimer) {
        clearInterval(testTimer);
        testTimer = null;
    }
    
    fetch('/test_mode', { method: 'DELETE' });
    alert('Test mode stopped');
}

// Data logging functions
function toggleLogging() {
    if (loggingActive) {
        stopLogging();
    } else {
        startLogging();
    }
}

function startLogging() {
    loggingActive = true;
    document.getElementById('logging-status').textContent = 'ACTIVE';
    document.getElementById('logging-btn').textContent = '⏸️ Stop Logging';
    document.getElementById('logging-btn').className = 'btn warning';
    
    const interval = parseInt(document.getElementById('log-interval').value);
    
    // Start logging timer
    const logTimer = setInterval(() => {
        if (!loggingActive) {
            clearInterval(logTimer);
            return;
        }
        addLogEntry();
    }, interval);
}

function stopLogging() {
    loggingActive = false;
    document.getElementById('logging-status').textContent = 'STOPPED';
    document.getElementById('logging-btn').textContent = '▶️ Start Logging';
    document.getElementById('logging-btn').className = 'btn';
}

function addLogEntry() {
    const now = new Date();
    const timestamp = now.toISOString().replace('T', ' ').substring(0, 19);
    
    // Get current values (these would come from ESP32 in real implementation)
    const mode = testModeActive ? 'TEST' : 'FIELD';
    const speed = testModeActive ? 
        document.getElementById('test-speed').value : 
        document.getElementById('current-speed').textContent.replace(' km/h', '');
    const rpm = document.getElementById('current-rpm').textContent;
    const pwm = document.getElementById('current-pwm').textContent;
    const satellites = document.getElementById('satellite-count').textContent;
    const hdop = '1.2'; // Would come from GPS
    const crop = selectedCrop ? selectedCrop.name : 'None';
    
    const entry = `${timestamp}, ${mode}, ${speed}, ${rpm}, ${pwm}, ${satellites}, ${hdop}, ${crop}`;
    logData.push(entry);
    
    // Update display
    const logDisplay = document.getElementById('log-display');
    const entryDiv = document.createElement('div');
    entryDiv.className = 'log-entry';
    entryDiv.textContent = entry;
    logDisplay.appendChild(entryDiv);
    
    // Keep only last 100 entries in display
    while (logDisplay.children.length > 101) { // 100 + header
        logDisplay.removeChild(logDisplay.children[1]); // Keep header
    }
    
    // Scroll to bottom
    logDisplay.scrollTop = logDisplay.scrollHeight;
    
    // Update count
    document.getElementById('log-count').textContent = logData.length;
}

function clearLog() {
    if (confirm('Clear all logged data?')) {
        logData = [];
        const logDisplay = document.getElementById('log-display');
        logDisplay.innerHTML = `
            <div class="log-entry">Timestamp, Mode, Speed(km/h), RPM, PWM, Satellites, HDOP, Crop</div>
            <div class="log-entry">--- No data logged yet ---</div>
        `;
        document.getElementById('log-count').textContent = '0';
    }
}

function downloadLog() {
    if (logData.length === 0) {
        alert('No data to download');
        return;
    }
    
    const csvContent = 'Timestamp,Mode,Speed(km/h),RPM,PWM,Satellites,HDOP,Crop\n' + 
                      logData.join('\n');
    
    const blob = new Blob([csvContent], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `seed_drill_log_${new Date().toISOString().substring(0, 10)}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);
}

// Control functions
function applyConfiguration() {
    if (!selectedCrop) {
        alert('Please select a crop first!');
        return;
    }
    
    const config = {
        crop: selectedCrop.id,
        spacing: selectedCrop.spacing,
        seedRate: selectedCrop.rate,
        speed: parseFloat(document.getElementById('speed-slider').value),
        soilType: document.getElementById('soil-type').value,
        season: document.getElementById('season').value
    };
    
    fetch('/apply-config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config)
    })
    .then(r => r.text())
    .then(result => {
        alert('Configuration applied successfully!\n' + result);
        updateStatus();
    })
    .catch(e => {
        alert('Failed to apply configuration: ' + e);
    });
}

function startCalibration() {
    if (confirm('Start motor calibration? This will take several minutes and requires manual RPM measurement.')) {
        fetch('/calibrate', { method: 'POST' })
        .then(r => r.text())
        .then(result => alert('Calibration: ' + result))
        .catch(e => alert('Calibration failed: ' + e));
    }
}

function emergencyStop() {
    fetch('/emergency', { method: 'POST' })
    .then(r => r.text())
    .then(result => {
        alert('Emergency Stop Activated!');
        updateStatus();
        stopTestMode();
    })
    .catch(e => alert('Emergency stop failed: ' + e));
}

// Settings functions
function performDiagnostics() {
    fetch('/diagnostics', { method: 'GET' })
    .then(r => r.text())
    .then(result => {
        alert('Diagnostics Results:\n' + result);
    })
    .catch(e => alert('Diagnostics failed: ' + e));
}

function exportSettings() {
    const settings = {
        selectedCrop: selectedCrop,
        soilType: document.getElementById('soil-type').value,
        season: document.getElementById('season').value,
        workingSpeed: document.getElementById('speed-slider').value,
        autoRefresh: document.getElementById('auto-refresh').checked,
        refreshRate: document.getElementById('refresh-rate').value,
        logInterval: document.getElementById('log-interval').value
    };
    
    const blob = new Blob([JSON.stringify(settings, null, 2)], { type: 'application/json' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `seed_drill_settings_${new Date().toISOString().substring(0, 10)}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);
}

function factoryReset() {
    if (confirm('Factory reset will clear all settings and calibration data. Continue?')) {
        fetch('/factory_reset', { method: 'POST' })
        .then(r => r.text())
        .then(result => {
            alert('Factory reset completed. System will restart.');
            location.reload();
        })
        .catch(e => alert('Factory reset failed: ' + e));
    }
}

function reconnect() {
    updateStatus();
    alert('Reconnection attempted');
}

// Status update functions
function updateStatus() {
    fetch('/status')
    .then(r => r.json())
    .then(data => {
        // Update system status
        document.getElementById('system-status').textContent = data.system_ready ? 'READY' : 'NOT READY';
        document.getElementById('gps-status').textContent = data.gps_locked ? 
            `Locked (${data.satellites} sats)` : 'Searching...';
        document.getElementById('motor-status').textContent = data.motor_running ? 'RUNNING' : 'STOPPED';
        
        // Update real-time data
        document.getElementById('current-speed').textContent = (data.speed || 0).toFixed(1) + ' km/h';
        document.getElementById('current-rpm').textContent = data.rpm || 0;
        document.getElementById('current-pwm').textContent = data.pwm || 0;
        document.getElementById('satellite-count').textContent = data.satellites || 0;
        
        // Update connection status
        document.getElementById('connection-status').textContent = 'Connected';
        document.getElementById('connection-status').style.color = '#28a745';
    })
    .catch(e => {
        console.log('Status update error:', e);
        document.getElementById('connection-status').textContent = 'Disconnected';
        document.getElementById('connection-status').style.color = '#dc3545';
    });
}

function startAutoRefresh() {
    const refreshRate = parseInt(document.getElementById('refresh-rate').value) || 2000;
    
    if (refreshTimer) {
        clearInterval(refreshTimer);
    }
    
    if (document.getElementById('auto-refresh').checked) {
        refreshTimer = setInterval(updateStatus, refreshRate);
    }
}

// Event listeners for settings
document.getElementById('auto-refresh').addEventListener('change', startAutoRefresh);
document.getElementById('refresh-rate').addEventListener('change', startAutoRefresh);

// Initialize speed slider
document.getElementById('speed-slider').addEventListener('input', updateSpeedDisplay);

// Keyboard shortcuts
document.addEventListener('keydown', function(e) {
    if (e.ctrlKey || e.metaKey) {
        switch(e.key) {
            case 'e':
                e.preventDefault();
                emergencyStop();
                break;
            case 's':
                e.preventDefault();
                if (e.shiftKey) {
                    stopTestMode();
                } else {
                    startTestMode();
                }
                break;
            case 'd':
                e.preventDefault();
                downloadLog();
                break;
        }
    }
});

// Window beforeunload handler
window.addEventListener('beforeunload', function(e) {
    if (testModeActive || loggingActive) {
        e.preventDefault();
        e.returnValue = '';
        return 'Test mode or logging is active. Are you sure you want to leave?';
    }
});

// Initialize on load
updateCalculations();
</script>
</body>
</html>
)rawliteral";

void setupEnhancedWebServer(AsyncWebServer& server);
void handleCropConfiguration(AsyncWebServerRequest *request);

#endif // ENHANCED_WEBINTERFACE_H
