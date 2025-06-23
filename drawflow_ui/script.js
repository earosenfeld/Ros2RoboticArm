// Global variables
let editor = null;
let selectedNode = null;
let websocket = null;
let isExecuting = false;

// API configuration
const API_BASE_URL = 'http://localhost:8000';

// Node type definitions with their properties
const NODE_TYPES = {
    MoveToPose: {
        props: {
            pose: {
                type: 'select',
                value: 'home',
                options: ['home', 'inspection', 'grasp', 'release', 'custom']
            },
            description: {
                type: 'text',
                value: 'Move to specified pose'
            }
        }
    },
    SetGripper: {
        props: {
            position: {
                type: 'select',
                value: '1.0 (Open)',
                options: ['0.0 (Closed)', '0.5 (Half)', '1.0 (Open)']
            },
            description: {
                type: 'text',
                value: 'Control gripper position'
            }
        }
    },
    CaptureImage: {
        props: {
            save: {
                type: 'checkbox',
                value: true
            },
            description: {
                type: 'text',
                value: 'Capture image from camera'
            }
        }
    },
    RunInspection: {
        props: {
            type: {
                type: 'select',
                value: 'standard',
                options: ['standard', 'detailed', 'quick']
            },
            description: {
                type: 'text',
                value: 'Execute inspection routine'
            }
        }
    }
};

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initializeDrawflow();
    initializeEventListeners();
    initializeWebSocket();
    // loadExampleWorkflow(); // Commented out to prevent errors on load
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
    
    // Handle node selection
    editor.on('nodeSelected', function(id) {
        console.log('Node selected:', id);
        selectedNode = id;
        showNodeConfiguration(id);
    });
    
    // Handle node creation
    editor.on('nodeCreated', function(id) {
        console.log('Node created:', id);
        // Log the node details
        const node = editor.getNodeFromId(id);
        console.log('Node details:', node);
    });
    
    // Handle connection creation
    editor.on('connectionCreated', function(connection) {
        console.log('Connection created:', connection);
    });
    
    // Handle connection error
    editor.on('connectionError', function(error) {
        console.error('Connection error:', error);
    });
    
    console.log('Drawflow initialized successfully');
    console.log('Editor object:', editor);
}

// Initialize event listeners
function initializeEventListeners() {
    // Header buttons
    document.getElementById('testBtn').addEventListener('click', createTestWorkflow);
    document.getElementById('saveBtn').addEventListener('click', saveWorkflow);
    document.getElementById('loadBtn').addEventListener('click', loadWorkflow);
    document.getElementById('executeBtn').addEventListener('click', executeWorkflow);
    document.getElementById('clearBtn').addEventListener('click', clearWorkflow);
    document.getElementById('visualizerBtn').addEventListener('click', openRobotVisualizer);
    
    // Add test connections button if it exists
    const testConnectionsBtn = document.getElementById('testConnectionsBtn');
    if (testConnectionsBtn) {
        testConnectionsBtn.addEventListener('click', testConnections);
    }
    
    // Add inspect nodes button if it exists
    const inspectNodesBtn = document.getElementById('inspectNodesBtn');
    if (inspectNodesBtn) {
        inspectNodesBtn.addEventListener('click', inspectNodes);
    }
    
    // Add basic test button if it exists
    const basicTestBtn = document.getElementById('basicTestBtn');
    if (basicTestBtn) {
        basicTestBtn.addEventListener('click', createBasicTest);
    }
    
    // Node palette drag and drop
    const nodeItems = document.querySelectorAll('.node-item');
    nodeItems.forEach(item => {
        item.addEventListener('dragstart', handleDragStart);
    });
    
    // Canvas drop zone
    const canvasContainer = document.querySelector('.canvas-container');
    canvasContainer.addEventListener('dragover', handleDragOver);
    canvasContainer.addEventListener('drop', handleDrop);
    
    // Modal events
    const modal = document.getElementById('nodeModal');
    const closeBtn = document.querySelector('.close');
    const saveNodeBtn = document.getElementById('saveNodeBtn');
    const cancelNodeBtn = document.getElementById('cancelNodeBtn');
    
    closeBtn.addEventListener('click', function(event) {
        event.stopPropagation(); // Prevent event from bubbling up to node
        closeModal();
    });
    saveNodeBtn.addEventListener('click', function(event) {
        event.stopPropagation(); // Prevent event from bubbling up to node
        saveNodeConfiguration();
    });
    cancelNodeBtn.addEventListener('click', function(event) {
        event.stopPropagation(); // Prevent event from bubbling up to node
        closeModal();
    });
    
    // Close modal when clicking outside
    window.addEventListener('click', function(event) {
        if (event.target === modal) {
            closeModal();
        }
    });
    
    // Prevent clicks inside modal from bubbling up to node
    modal.addEventListener('click', function(event) {
        event.stopPropagation();
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
    console.log('Drag started for node:', event.target.dataset.node);
    event.dataTransfer.setData('text/plain', event.target.dataset.node);
}

function handleDragOver(event) {
    event.preventDefault();
}

function handleDrop(event) {
    event.preventDefault();
    const nodeType = event.dataTransfer.getData('text/plain');
    console.log('Drop event triggered for node type:', nodeType);
    
    // Get the canvas container element
    const canvasContainer = document.querySelector('.canvas-container');
    const rect = canvasContainer.getBoundingClientRect();
    
    // Calculate position relative to the canvas container
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    console.log('Drop position:', { x, y, clientX: event.clientX, clientY: event.clientY, rect });
    
    createNode(nodeType, x, y);
}

// Create a new node
function createNode(nodeType, x, y) {
    try {
        let htmlContent = '';
        let nodeData = {};
        let nodeProps = {};
        
        // Get node type definition
        const nodeTypeDef = NODE_TYPES[nodeType];
        if (nodeTypeDef) {
            nodeProps = nodeTypeDef.props;
            // Initialize nodeData with default values from props
            for (const [key, prop] of Object.entries(nodeProps)) {
                nodeData[key] = prop.value;
            }
        }
        
        // Check if this is the first node (no other nodes exist)
        const allNodes = editor.getNodesFromName();
        const isFirstNode = Object.keys(allNodes).length === 0;
        
        console.log(`Creating ${nodeType} node. Is first node: ${isFirstNode}`);
        
        switch (nodeType) {
            case 'MoveToPose':
                htmlContent = `
                    <div style="padding: 10px; background: white; border: 1px solid #ccc; border-radius: 8px;">
                        <div class="title-box" style="background: #667eea; color: white; padding: 5px; border-radius: 4px; margin-bottom: 10px;">Move To Pose</div>
                        <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                            ${!isFirstNode ? '<div class="input-box" data-input="input" style="width: 20px; height: 20px; background: #e9ecef; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>' : '<div style="width: 20px;"></div>'}
                            <div class="output-box" data-output="output" style="width: 20px; height: 20px; background: #667eea; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>
                        </div>
                    </div>
                `;
                break;
            case 'SetGripper':
                htmlContent = `
                    <div style="padding: 10px; background: white; border: 1px solid #ccc; border-radius: 8px;">
                        <div class="title-box" style="background: #667eea; color: white; padding: 5px; border-radius: 4px; margin-bottom: 10px;">Set Gripper</div>
                        <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                            ${!isFirstNode ? '<div class="input-box" data-input="input" style="width: 20px; height: 20px; background: #e9ecef; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>' : '<div style="width: 20px;"></div>'}
                            <div class="output-box" data-output="output" style="width: 20px; height: 20px; background: #667eea; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>
                        </div>
                    </div>
                `;
                break;
            case 'CaptureImage':
                htmlContent = `
                    <div style="padding: 10px; background: white; border: 1px solid #ccc; border-radius: 8px;">
                        <div class="title-box" style="background: #667eea; color: white; padding: 5px; border-radius: 4px; margin-bottom: 10px;">Capture Image</div>
                        <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                            ${!isFirstNode ? '<div class="input-box" data-input="input" style="width: 20px; height: 20px; background: #e9ecef; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>' : '<div style="width: 20px;"></div>'}
                            <div class="output-box" data-output="output" style="width: 20px; height: 20px; background: #667eea; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>
                        </div>
                    </div>
                `;
                break;
            case 'RunInspection':
                htmlContent = `
                    <div style="padding: 10px; background: white; border: 1px solid #ccc; border-radius: 8px;">
                        <div class="title-box" style="background: #667eea; color: white; padding: 5px; border-radius: 4px; margin-bottom: 10px;">Run Inspection</div>
                        <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                            ${!isFirstNode ? '<div class="input-box" data-input="input" style="width: 20px; height: 20px; background: #e9ecef; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>' : '<div style="width: 20px;"></div>'}
                            <div class="output-box" data-output="output" style="width: 20px; height: 20px; background: #667eea; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>
                        </div>
                    </div>
                `;
                break;
            default:
                htmlContent = `
                    <div style="padding: 10px; background: white; border: 1px solid #ccc; border-radius: 8px;">
                        <div class="title-box" style="background: #667eea; color: white; padding: 5px; border-radius: 4px; margin-bottom: 10px;">${nodeType}</div>
                        <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                            ${!isFirstNode ? '<div class="input-box" data-input="input" style="width: 20px; height: 20px; background: #e9ecef; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>' : '<div style="width: 20px;"></div>'}
                            <div class="output-box" data-output="output" style="width: 20px; height: 20px; background: #667eea; border-radius: 50%; cursor: crosshair; border: 2px solid white;"></div>
                        </div>
                    </div>
                `;
                nodeData = { description: `Default ${nodeType} description` };
        }

        // For Drawflow 0.0.59, the correct signature is:
        // addNode(name, inputs, outputs, pos_x, pos_y, class, data, html)
        const nodeId = editor.addNode(
            nodeType, // name
            isFirstNode ? 0 : 1, // inputs: 0 for first node, 1 for others
            1,        // outputs
            x,        // pos_x
            y,        // pos_y
            'robot',  // class
            nodeData, // data
            htmlContent // html
        );
        
        // Set the props property on the node after creation
        if (nodeId && nodeProps) {
            const node = editor.getNodeFromId(nodeId);
            if (node) {
                node.props = nodeProps;
            }
        }
        
        console.log(`Created ${nodeType} node with ID: ${nodeId} (inputs: ${isFirstNode ? 0 : 1})`);
        return nodeId;
    } catch (error) {
        console.error('Error creating node:', error);
        console.error('Node creation details:', { nodeType, x, y });
        showToast('Failed to create node', 'error');
        return null;
    }
}

// Test function to create a simple workflow
function createTestWorkflow() {
    console.log('Creating test workflow...');
    
    // Clear existing workflow
    editor.clear();
    
    // Create nodes with proper spacing
    const node1 = createNode('MoveToPose', 100, 100);
    const node2 = createNode('SetGripper', 350, 100);
    const node3 = createNode('CaptureImage', 600, 100);
    
    console.log('Created nodes:', { node1, node2, node3 });
    
    // Wait a bit for nodes to be fully created
    setTimeout(() => {
        // Connect nodes
        if (node1 && node2) {
            try {
                editor.addConnection(node1, 'output', node2, 'input');
                console.log('Connected node1 to node2');
            } catch (error) {
                console.error('Failed to connect node1 to node2:', error);
            }
        }
        
        if (node2 && node3) {
            try {
                editor.addConnection(node2, 'output', node3, 'input');
                console.log('Connected node2 to node3');
            } catch (error) {
                console.error('Failed to connect node2 to node3:', error);
            }
        }
        
        // Export and test
        const workflowData = editor.export();
        console.log('Test workflow data:', workflowData);
        
        const commands = convertWorkflowToCommands(workflowData);
        console.log('Test workflow commands:', commands);
        
        showToast('Test workflow created! Check console for details.', 'success');
    }, 500);
}

// Debug function to test connections manually
function testConnections() {
    console.log('Testing connections...');
    
    // Get all nodes
    const nodes = editor.getNodesFromName();
    console.log('All nodes:', nodes);
    
    // Try to get the first two nodes and connect them
    const nodeIds = Object.keys(nodes);
    if (nodeIds.length >= 2) {
        const node1Id = nodeIds[0];
        const node2Id = nodeIds[1];
        
        console.log(`Attempting to connect ${node1Id} to ${node2Id}`);
        
        try {
            editor.addConnection(node1Id, 'output', node2Id, 'input');
            console.log('Connection successful!');
            showToast('Connection test successful!', 'success');
        } catch (error) {
            console.error('Connection failed:', error);
            showToast('Connection test failed: ' + error.message, 'error');
        }
    } else {
        console.log('Need at least 2 nodes to test connections');
        showToast('Need at least 2 nodes to test connections', 'info');
    }
}

// Debug function to inspect node structure
function inspectNodes() {
    console.log('Inspecting nodes...');
    
    const nodes = editor.getNodesFromName();
    console.log('All nodes:', nodes);
    
    for (const [nodeId, node] of Object.entries(nodes)) {
        console.log(`Node ${nodeId}:`, {
            id: node.id,
            name: node.name,
            inputs: node.inputs,
            outputs: node.outputs,
            html: node.html,
            data: node.data
        });
    }
    
    showToast('Node inspection complete - check console', 'info');
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
    
    // Safety check: if node.props is undefined, try to get it from NODE_TYPES
    let nodeProps = node.props;
    if (!nodeProps && NODE_TYPES[node.name]) {
        nodeProps = NODE_TYPES[node.name].props;
        // Also set it on the node for future use
        node.props = nodeProps;
    }
    
    if (nodeProps) {
        for (const [key, prop] of Object.entries(nodeProps)) {
            formHTML += generateFormField(key, prop, node.data[key]);
        }
    } else {
        // Fallback: show basic configuration for unknown node types
        formHTML = `
            <div class="form-group">
                <label for="description">Description:</label>
                <input type="text" id="description" value="${node.data.description || ''}" onclick="event.stopPropagation()" onmousedown="event.stopPropagation()" />
            </div>
        `;
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
                    <input type="text" id="${key}" value="${value || prop.value || ''}" onclick="event.stopPropagation()" onmousedown="event.stopPropagation()" />
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
                    <select id="${key}" onclick="event.stopPropagation()" onmousedown="event.stopPropagation()">${options}</select>
                </div>
            `;
        case 'checkbox':
            const checked = value ? 'checked' : '';
            return `
                <div class="form-group">
                    <label for="${key}">
                        <input type="checkbox" id="${key}" ${checked} onclick="event.stopPropagation()" onmousedown="event.stopPropagation()" />
                        ${label}
                    </label>
                </div>
            `;
        default:
            return `
                <div class="form-group">
                    <label for="${key}">${label}:</label>
                    <input type="text" id="${key}" value="${value || prop.value || ''}" onclick="event.stopPropagation()" onmousedown="event.stopPropagation()" />
                </div>
            `;
    }
}

// Save node configuration
function saveNodeConfiguration() {
    if (!selectedNode) return;
    
    const node = editor.getNodeFromId(selectedNode);
    if (!node) return;
    
    // Safety check: if node.props is undefined, try to get it from NODE_TYPES
    let nodeProps = node.props;
    if (!nodeProps && NODE_TYPES[node.name]) {
        nodeProps = NODE_TYPES[node.name].props;
        // Also set it on the node for future use
        node.props = nodeProps;
    }
    
    // Collect form values
    if (nodeProps) {
        for (const [key, prop] of Object.entries(nodeProps)) {
            const element = document.getElementById(key);
            if (element) {
                if (prop.type === 'checkbox') {
                    node.data[key] = element.checked;
                } else {
                    node.data[key] = element.value;
                }
            }
        }
    } else {
        // Fallback: save basic description for unknown node types
        const descriptionElement = document.getElementById('description');
        if (descriptionElement) {
            node.data.description = descriptionElement.value;
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
    if (isExecuting) return;
    
    try {
        isExecuting = true;
        updateExecutionStatus('executing');
        
        // Export workflow data
        const workflowData = editor.export();
        console.log('Exported workflow data:', workflowData);
        
        // Convert to ROS 2 commands
        const commands = convertWorkflowToCommands(workflowData);
        console.log('Converted commands:', commands);
        
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
    console.log('Converting workflow data:', workflowData);
    const commands = [];
    
    // Check if workflowData has the expected structure
    if (!workflowData || !workflowData.drawflow) {
        console.warn('No drawflow property in workflow data');
        return commands;
    }
    
    const nodes = workflowData.drawflow.Home?.data;
    console.log('Nodes from workflow:', nodes);
    
    if (!nodes || Object.keys(nodes).length === 0) {
        console.warn('No nodes found in workflow');
        return commands;
    }
    
    // Find start nodes (nodes with no incoming connections)
    const startNodes = Object.values(nodes).filter(node => {
        // Check if this node has any incoming connections
        const hasIncomingConnections = Object.values(nodes).some(otherNode => {
            if (otherNode.outputs) {
                for (const outputKey in otherNode.outputs) {
                    const output = otherNode.outputs[outputKey];
                    if (output.connections) {
                        for (const connection of output.connections) {
                            if (connection.node === node.id.toString()) {
                                return true; // This node has an incoming connection
                            }
                        }
                    }
                }
            }
            return false;
        });
        
        const isStartNode = !hasIncomingConnections;
        console.log(`Node ${node.id} (${node.name}): hasIncomingConnections = ${hasIncomingConnections}, isStartNode = ${isStartNode}`);
        return isStartNode;
    });
    
    console.log('Start nodes found:', startNodes);
    
    if (startNodes.length === 0) {
        console.warn('No start nodes found in workflow');
        return commands;
    }
    
    // Process nodes in topological order
    const processed = new Set();
    const queue = [...startNodes];
    
    while (queue.length > 0) {
        const node = queue.shift();
        const nodeId = node.id.toString();
        
        if (processed.has(nodeId)) {
            continue;
        }
        
        // Convert node to command
        const command = convertNodeToCommand(node);
        if (command) {
            commands.push(command);
        }
        
        processed.add(nodeId);
        
        // Add connected output nodes to queue
        if (node.outputs) {
            for (const outputKey in node.outputs) {
                const output = node.outputs[outputKey];
                if (output.connections) {
                    for (const connection of output.connections) {
                        const nextNodeId = connection.node;
                        const nextNode = nodes[nextNodeId];
                        if (nextNode && !processed.has(nextNodeId)) {
                            // Check if all inputs of this node are processed
                            const allInputsProcessed = !nextNode.inputs || 
                                Object.values(nextNode.inputs).every(input => 
                                    !input.connections || 
                                    input.connections.every(conn => processed.has(conn.node))
                                );
                            
                            if (allInputsProcessed) {
                                queue.push(nextNode);
                            }
                        }
                    }
                }
            }
        }
    }
    
    console.log(`Converted ${commands.length} commands from workflow`);
    return commands;
}

// Convert a single node to a command
function convertNodeToCommand(node) {
    console.log('Converting node:', node);
    
    const baseCommand = {
        type: node.name,
        description: node.data?.description || ''
    };
    
    switch (node.name) {
        case 'MoveToPose':
            return {
                ...baseCommand,
                pose: node.data?.pose || 'home'
            };
        case 'SetGripper':
            const positionStr = node.data?.position || '0.0 (Closed)';
            // Extract numeric value from position string
            const positionMatch = positionStr.match(/(\d+\.?\d*)/);
            const position = positionMatch ? parseFloat(positionMatch[1]) : 0.0;
            return {
                ...baseCommand,
                position: position
            };
        case 'CaptureImage':
            return {
                ...baseCommand,
                save: node.data?.save !== undefined ? node.data.save : true
            };
        case 'RunInspection':
            return {
                ...baseCommand,
                type: node.data?.type || 'standard'
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
                        pos_y: 100,
                        width: 150,
                        height: 80
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
                        pos_y: 100,
                        width: 150,
                        height: 80
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
                        pos_y: 100,
                        width: 150,
                        height: 80
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
                        pos_y: 100,
                        width: 150,
                        height: 80
                    }
                }
            }
        }
    };
    
    try {
        editor.import(exampleData);
        showToast('Example workflow loaded', 'info');
    } catch (error) {
        console.error('Error loading example workflow:', error);
        showToast('Failed to load example workflow', 'error');
    }
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

// Simple test function to create basic nodes
function createBasicTest() {
    console.log('Creating basic test...');
    
    // Clear existing workflow
    editor.clear();
    
    // Create a simple test node with minimal HTML
    const testHtml = `
        <div style="padding: 10px; background: white; border: 1px solid #ccc;">
            <div>Test Node</div>
            <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                <div class="input-box" data-input="input" style="width: 20px; height: 20px; background: #e9ecef; border-radius: 50%; cursor: crosshair;"></div>
                <div class="output-box" data-output="output" style="width: 20px; height: 20px; background: #667eea; border-radius: 50%; cursor: crosshair;"></div>
            </div>
        </div>
    `;
    
    try {
        const node1 = editor.addNode('TestNode', 1, 1, 100, 100, 'test', { test: 'data' }, testHtml);
        const node2 = editor.addNode('TestNode', 1, 1, 300, 100, 'test', { test: 'data' }, testHtml);
        
        console.log('Created test nodes:', { node1, node2 });
        
        // Try to connect them
        if (node1 && node2) {
            editor.addConnection(node1, 'output', node2, 'input');
            console.log('Test connection successful!');
            showToast('Basic test successful!', 'success');
        }
        
    } catch (error) {
        console.error('Basic test failed:', error);
        showToast('Basic test failed: ' + error.message, 'error');
    }
} 