// Global variables
let editor = null;
let selectedNode = null;
let websocket = null;
let isExecuting = false;

// API configuration
const API_BASE_URL = 'http://localhost:8000';

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initializeDrawflow();
    initializeEventListeners();
    initializeWebSocket();
    loadExampleWorkflow();
});

// Initialize Drawflow editor
function initializeDrawflow() {
    const element = document.getElementById('drawflow');
    editor = new Drawflow(element);
    editor.reroute = true;
    editor.reroute_fix_curvature = true;
    editor.force_first_input = false;
    editor.line_path = 1;
    editor.start();
    
    // Register custom node types
    registerCustomNodes();
    
    // Handle node selection
    editor.on('nodeSelected', function(id) {
        selectedNode = id;
        showNodeConfiguration(id);
    });
    
    // Handle node creation
    editor.on('nodeCreated', function(id) {
        console.log('Node created:', id);
    });
    
    // Handle connection creation
    editor.on('connectionCreated', function(connection) {
        console.log('Connection created:', connection);
    });
}

// Register custom node types
function registerCustomNodes() {
    // MoveToPose node
    editor.registerNode('MoveToPose', {
        name: 'Move To Pose',
        category: 'robot',
        html: `
            <div class="title-box">Move To Pose</div>
            <div class="box">
                <div class="input-box" data-input="input"></div>
                <div class="output-box" data-output="output"></div>
            </div>
        `,
        props: {
            pose: { type: 'select', options: ['home', 'inspection_1', 'inspection_2', 'inspection_3'] },
            description: { type: 'text', value: 'Move to specified pose' }
        },
        inputs: 1,
        outputs: 1,
        icon: 'fas fa-arrows-alt'
    });
    
    // SetGripper node
    editor.registerNode('SetGripper', {
        name: 'Set Gripper',
        category: 'robot',
        html: `
            <div class="title-box">Set Gripper</div>
            <div class="box">
                <div class="input-box" data-input="input"></div>
                <div class="output-box" data-output="output"></div>
            </div>
        `,
        props: {
            position: { type: 'select', options: ['0.0 (Closed)', '1.0 (Open)'] },
            description: { type: 'text', value: 'Control gripper position' }
        },
        inputs: 1,
        outputs: 1,
        icon: 'fas fa-hand-paper'
    });
    
    // CaptureImage node
    editor.registerNode('CaptureImage', {
        name: 'Capture Image',
        category: 'camera',
        html: `
            <div class="title-box">Capture Image</div>
            <div class="box">
                <div class="input-box" data-input="input"></div>
                <div class="output-box" data-output="output"></div>
            </div>
        `,
        props: {
            save: { type: 'checkbox', value: true },
            description: { type: 'text', value: 'Capture image from camera' }
        },
        inputs: 1,
        outputs: 1,
        icon: 'fas fa-camera'
    });
    
    // RunInspection node
    editor.registerNode('RunInspection', {
        name: 'Run Inspection',
        category: 'inspection',
        html: `
            <div class="title-box">Run Inspection</div>
            <div class="box">
                <div class="input-box" data-input="input"></div>
                <div class="output-box" data-output="output"></div>
            </div>
        `,
        props: {
            type: { type: 'select', options: ['standard', 'detailed', 'quick'] },
            description: { type: 'text', value: 'Execute inspection routine' }
        },
        inputs: 1,
        outputs: 1,
        icon: 'fas fa-search'
    });
}

// Initialize event listeners
function initializeEventListeners() {
    // Header buttons
    document.getElementById('saveBtn').addEventListener('click', saveWorkflow);
    document.getElementById('loadBtn').addEventListener('click', loadWorkflow);
    document.getElementById('executeBtn').addEventListener('click', executeWorkflow);
    document.getElementById('clearBtn').addEventListener('click', clearWorkflow);
    document.getElementById('visualizerBtn').addEventListener('click', openRobotVisualizer);
    
    // Node palette drag and drop
    const nodeItems = document.querySelectorAll('.node-item');
    nodeItems.forEach(item => {
        item.addEventListener('dragstart', handleDragStart);
    });
    
    // Canvas drop zone
    const canvas = document.getElementById('drawflow');
    canvas.addEventListener('dragover', handleDragOver);
    canvas.addEventListener('drop', handleDrop);
    
    // Modal events
    const modal = document.getElementById('nodeModal');
    const closeBtn = document.querySelector('.close');
    const saveNodeBtn = document.getElementById('saveNodeBtn');
    const cancelNodeBtn = document.getElementById('cancelNodeBtn');
    
    closeBtn.addEventListener('click', closeModal);
    saveNodeBtn.addEventListener('click', saveNodeConfiguration);
    cancelNodeBtn.addEventListener('click', closeModal);
    
    // Close modal when clicking outside
    window.addEventListener('click', function(event) {
        if (event.target === modal) {
            closeModal();
        }
    });
}

// Initialize WebSocket connection
function initializeWebSocket() {
    try {
        websocket = new WebSocket(`ws://localhost:8000/ws/status`);
        
        websocket.onopen = function(event) {
            console.log('WebSocket connected');
            updateStatus('connected');
        };
        
        websocket.onmessage = function(event) {
            const data = JSON.parse(event.data);
            handleStatusUpdate(data);
        };
        
        websocket.onclose = function(event) {
            console.log('WebSocket disconnected');
            updateStatus('disconnected');
            // Try to reconnect after 5 seconds
            setTimeout(initializeWebSocket, 5000);
        };
        
        websocket.onerror = function(error) {
            console.error('WebSocket error:', error);
            updateStatus('error');
        };
    } catch (error) {
        console.error('Failed to initialize WebSocket:', error);
        showToast('WebSocket connection failed', 'error');
    }
}

// Handle drag and drop for node creation
function handleDragStart(event) {
    event.dataTransfer.setData('text/plain', event.target.dataset.node);
}

function handleDragOver(event) {
    event.preventDefault();
}

function handleDrop(event) {
    event.preventDefault();
    const nodeType = event.dataTransfer.getData('text/plain');
    const rect = event.target.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    createNode(nodeType, x, y);
}

// Create a new node
function createNode(nodeType, x, y) {
    const nodeId = editor.addNode(nodeType, 0, x, y, {}, 'robot');
    console.log(`Created ${nodeType} node with ID: ${nodeId}`);
}

// Show node configuration modal
function showNodeConfiguration(nodeId) {
    const node = editor.getNodeFromId(nodeId);
    if (!node) return;
    
    const modal = document.getElementById('nodeModal');
    const modalTitle = document.getElementById('modalTitle');
    const modalBody = document.getElementById('modalBody');
    
    modalTitle.textContent = `Configure ${node.name}`;
    
    // Generate form based on node properties
    let formHTML = '';
    for (const [key, prop] of Object.entries(node.props)) {
        formHTML += generateFormField(key, prop, node.data[key]);
    }
    
    modalBody.innerHTML = formHTML;
    modal.style.display = 'block';
}

// Generate form field HTML
function generateFormField(key, prop, value) {
    const label = key.charAt(0).toUpperCase() + key.slice(1);
    
    switch (prop.type) {
        case 'text':
            return `
                <div class="form-group">
                    <label for="${key}">${label}:</label>
                    <input type="text" id="${key}" value="${value || prop.value || ''}" />
                </div>
            `;
        case 'select':
            const options = prop.options.map(opt => {
                const selected = opt === value ? 'selected' : '';
                return `<option value="${opt}" ${selected}>${opt}</option>`;
            }).join('');
            return `
                <div class="form-group">
                    <label for="${key}">${label}:</label>
                    <select id="${key}">${options}</select>
                </div>
            `;
        case 'checkbox':
            const checked = value !== undefined ? value : prop.value;
            return `
                <div class="form-group">
                    <div class="checkbox-group">
                        <input type="checkbox" id="${key}" ${checked ? 'checked' : ''} />
                        <label for="${key}">${label}</label>
                    </div>
                </div>
            `;
        default:
            return '';
    }
}

// Save node configuration
function saveNodeConfiguration() {
    if (!selectedNode) return;
    
    const node = editor.getNodeFromId(selectedNode);
    if (!node) return;
    
    // Collect form values
    for (const [key, prop] of Object.entries(node.props)) {
        const element = document.getElementById(key);
        if (element) {
            if (prop.type === 'checkbox') {
                node.data[key] = element.checked;
            } else {
                node.data[key] = element.value;
            }
        }
    }
    
    // Update node display
    editor.updateNodeDataFromId(selectedNode, node.data);
    
    closeModal();
    showToast('Node configuration saved', 'success');
}

// Close modal
function closeModal() {
    document.getElementById('nodeModal').style.display = 'none';
    selectedNode = null;
}

// Save workflow
function saveWorkflow() {
    const data = editor.export();
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    
    const a = document.createElement('a');
    a.href = url;
    a.download = `robot_inspection_workflow_${Date.now()}.json`;
    a.click();
    
    URL.revokeObjectURL(url);
    showToast('Workflow saved successfully', 'success');
}

// Load workflow
function loadWorkflow() {
    document.getElementById('fileInput').click();
}

// Handle file input change
document.getElementById('fileInput').addEventListener('change', function(event) {
    const file = event.target.files[0];
    if (file) {
        const reader = new FileReader();
        reader.onload = function(e) {
            try {
                const data = JSON.parse(e.target.result);
                editor.import(data);
                showToast('Workflow loaded successfully', 'success');
            } catch (error) {
                showToast('Failed to load workflow', 'error');
                console.error('Error loading workflow:', error);
            }
        };
        reader.readAsText(file);
    }
});

// Execute workflow
async function executeWorkflow() {
    if (isExecuting) {
        showToast('Workflow already executing', 'error');
        return;
    }
    
    try {
        isExecuting = true;
        updateExecutionStatus('executing');
        
        // Export workflow data
        const workflowData = editor.export();
        
        // Convert to ROS 2 commands
        const commands = convertWorkflowToCommands(workflowData);
        
        // Send to API
        const response = await fetch(`${API_BASE_URL}/api/execute-routine`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                name: 'Drawflow Workflow',
                steps: commands
            })
        });
        
        if (response.ok) {
            const result = await response.json();
            showToast(`Workflow started: ${result.message}`, 'success');
        } else {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        
    } catch (error) {
        console.error('Error executing workflow:', error);
        showToast(`Execution failed: ${error.message}`, 'error');
    } finally {
        isExecuting = false;
        updateExecutionStatus('idle');
    }
}

// Convert workflow to ROS 2 commands
function convertWorkflowToCommands(workflowData) {
    const commands = [];
    const nodes = workflowData.drawflow.Home.data;
    
    // Find start nodes (nodes with no inputs or empty inputs)
    const startNodes = Object.values(nodes).filter(node => 
        !node.inputs || Object.keys(node.inputs).length === 0
    );
    
    // Process nodes in order (simplified - in real implementation, you'd need proper topological sorting)
    for (const node of startNodes) {
        const command = convertNodeToCommand(node);
        if (command) {
            commands.push(command);
        }
    }
    
    return commands;
}

// Convert a single node to a command
function convertNodeToCommand(node) {
    const baseCommand = {
        type: node.name,
        description: node.data.description || ''
    };
    
    switch (node.name) {
        case 'MoveToPose':
            return {
                ...baseCommand,
                pose: node.data.pose || 'home'
            };
        case 'SetGripper':
            return {
                ...baseCommand,
                position: parseFloat(node.data.position) || 0.0
            };
        case 'CaptureImage':
            return {
                ...baseCommand,
                save: node.data.save !== undefined ? node.data.save : true
            };
        case 'RunInspection':
            return {
                ...baseCommand,
                type: node.data.type || 'standard'
            };
        default:
            console.warn(`Unknown node type: ${node.name}`);
            return null;
    }
}

// Clear workflow
function clearWorkflow() {
    if (confirm('Are you sure you want to clear the entire workflow?')) {
        editor.clear();
        showToast('Workflow cleared', 'info');
    }
}

// Handle status updates from WebSocket
function handleStatusUpdate(data) {
    console.log('Status update:', data);
    
    if (data.type === 'robot_status') {
        updateRobotStatus(data.status);
    } else if (data.type === 'inspection_result') {
        addInspectionResult(data.data);
    } else if (data.status === 'completed') {
        updateExecutionStatus('completed');
        showToast('Workflow execution completed', 'success');
    }
}

// Update robot status
function updateRobotStatus(status) {
    const statusElement = document.getElementById('robotStatus');
    statusElement.textContent = status;
    statusElement.className = 'status-value ' + (status === 'Ready' ? 'ready' : 'executing');
}

// Update execution status
function updateExecutionStatus(status) {
    const statusElement = document.getElementById('executionStatus');
    statusElement.textContent = status;
    statusElement.className = 'status-value ' + status;
}

// Add inspection result
function addInspectionResult(result) {
    const resultsPanel = document.getElementById('resultsPanel');
    const noResults = resultsPanel.querySelector('.no-results');
    
    if (noResults) {
        noResults.remove();
    }
    
    const resultItem = document.createElement('div');
    resultItem.className = `result-item ${result.overall_result === 'PASS' ? 'success' : 'error'}`;
    
    resultItem.innerHTML = `
        <div class="result-title">${result.overall_result} - ${new Date(result.timestamp).toLocaleTimeString()}</div>
        <div class="result-details">Defects found: ${result.defects_found}, Confidence: ${(result.confidence * 100).toFixed(1)}%</div>
    `;
    
    resultsPanel.insertBefore(resultItem, resultsPanel.firstChild);
    
    // Keep only last 5 results
    const results = resultsPanel.querySelectorAll('.result-item');
    if (results.length > 5) {
        results[results.length - 1].remove();
    }
}

// Update connection status
function updateStatus(status) {
    const cameraStatus = document.getElementById('cameraStatus');
    cameraStatus.textContent = status === 'connected' ? 'Connected' : 'Disconnected';
    cameraStatus.className = 'status-value ' + (status === 'connected' ? 'connected' : 'disconnected');
}

// Show toast notification
function showToast(message, type = 'info') {
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    
    document.body.appendChild(toast);
    
    setTimeout(() => {
        toast.remove();
    }, 3000);
}

// Load example workflow
function loadExampleWorkflow() {
    const exampleData = {
        drawflow: {
            Home: {
                data: {
                    "1": {
                        id: 1,
                        name: "MoveToPose",
                        data: { pose: "home", description: "Move to home position" },
                        class: "MoveToPose",
                        html: "",
                        typenode: false,
                        inputs: {},
                        outputs: { output: { connections: [{ node: "2", input: "input" }] } },
                        pos_x: 100,
                        pos_y: 100
                    },
                    "2": {
                        id: 2,
                        name: "SetGripper",
                        data: { position: "1.0 (Open)", description: "Open gripper" },
                        class: "SetGripper",
                        html: "",
                        typenode: false,
                        inputs: { input: { connections: [{ node: "1", output: "output" }] } },
                        outputs: { output: { connections: [{ node: "3", input: "input" }] } },
                        pos_x: 300,
                        pos_y: 100
                    },
                    "3": {
                        id: 3,
                        name: "CaptureImage",
                        data: { save: true, description: "Capture image" },
                        class: "CaptureImage",
                        html: "",
                        typenode: false,
                        inputs: { input: { connections: [{ node: "2", output: "output" }] } },
                        outputs: { output: { connections: [{ node: "4", input: "input" }] } },
                        pos_x: 500,
                        pos_y: 100
                    },
                    "4": {
                        id: 4,
                        name: "RunInspection",
                        data: { type: "standard", description: "Run inspection" },
                        class: "RunInspection",
                        html: "",
                        typenode: false,
                        inputs: { input: { connections: [{ node: "3", output: "output" }] } },
                        outputs: {},
                        pos_x: 700,
                        pos_y: 100
                    }
                }
            }
        }
    };
    
    editor.import(exampleData);
    showToast('Example workflow loaded', 'info');
}

// Open 3D robot visualizer
function openRobotVisualizer() {
    const visualizerUrl = window.location.origin + '/robot_visualizer.html';
    const visualizerWindow = window.open(visualizerUrl, 'robot_visualizer', 
        'width=1200,height=800,scrollbars=yes,resizable=yes');
    
    if (visualizerWindow) {
        showToast('3D Robot Visualizer opened in new window', 'info');
    } else {
        showToast('Failed to open 3D Robot Visualizer. Please allow popups.', 'error');
    }
} 