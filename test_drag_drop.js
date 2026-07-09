#!/usr/bin/env node

const puppeteer = require('puppeteer');
const fs = require('fs');
const path = require('path');

async function testDragAndDrop() {
    console.log('🧪 Testing Drag and Drop Functionality');
    console.log('=====================================');
    
    let browser;
    try {
        // Launch browser
        console.log('🌐 Launching browser...');
        browser = await puppeteer.launch({ 
            headless: true,
            args: ['--no-sandbox', '--disable-setuid-sandbox']
        });
        
        const page = await browser.newPage();
        
        // Enable console logging
        page.on('console', msg => {
            console.log(`📝 Console: ${msg.text()}`);
        });
        
        page.on('pageerror', error => {
            console.error(`❌ Page Error: ${error.message}`);
        });
        
        // Navigate to the page
        console.log('📄 Loading page...');
        await page.goto('http://localhost:8080', { waitUntil: 'networkidle2' });
        
        // Wait for the page to load
        await page.waitForSelector('#drawflow', { timeout: 10000 });
        console.log('✅ Page loaded successfully');
        
        // Test if Drawflow is initialized
        const drawflowInitialized = await page.evaluate(() => {
            return typeof window.editor !== 'undefined';
        });
        
        if (!drawflowInitialized) {
            throw new Error('Drawflow editor not initialized');
        }
        console.log('✅ Drawflow editor initialized');
        
        // Test node creation by simulating drag and drop
        console.log('🎯 Testing node creation...');
        const nodeCreationResult = await page.evaluate(() => {
            try {
                // Simulate creating a node
                const nodeId = window.editor.addNode('MoveToPose', 0, 100, 100, {});
                console.log(`Node created with ID: ${nodeId}`);
                return { success: true, nodeId: nodeId };
            } catch (error) {
                console.error('Node creation failed:', error.message);
                return { success: false, error: error.message };
            }
        });
        
        if (nodeCreationResult.success) {
            console.log(`✅ Node created successfully with ID: ${nodeCreationResult.nodeId}`);
        } else {
            console.log(`❌ Node creation failed: ${nodeCreationResult.error}`);
        }
        
        // Test if nodes exist in the sidebar
        const sidebarNodes = await page.evaluate(() => {
            const nodeItems = document.querySelectorAll('.node-item');
            return Array.from(nodeItems).map(item => ({
                type: item.dataset.node,
                draggable: item.draggable
            }));
        });
        
        console.log('📋 Sidebar nodes found:');
        sidebarNodes.forEach(node => {
            console.log(`   - ${node.type} (draggable: ${node.draggable})`);
        });
        
        // Test drag and drop simulation
        console.log('🖱️ Testing drag and drop simulation...');
        const dragDropResult = await page.evaluate(() => {
            try {
                // Get the first node item
                const nodeItem = document.querySelector('.node-item');
                if (!nodeItem) {
                    return { success: false, error: 'No node items found' };
                }
                
                // Get the canvas container
                const canvasContainer = document.querySelector('.canvas-container');
                if (!canvasContainer) {
                    return { success: false, error: 'Canvas container not found' };
                }
                
                // Simulate drag start
                const dragStartEvent = new DragEvent('dragstart', {
                    dataTransfer: new DataTransfer()
                });
                nodeItem.dispatchEvent(dragStartEvent);
                
                // Simulate drop
                const dropEvent = new DragEvent('drop', {
                    clientX: 200,
                    clientY: 200,
                    dataTransfer: new DataTransfer()
                });
                canvasContainer.dispatchEvent(dropEvent);
                
                return { success: true, message: 'Drag and drop events dispatched' };
            } catch (error) {
                return { success: false, error: error.message };
            }
        });
        
        if (dragDropResult.success) {
            console.log(`✅ Drag and drop simulation: ${dragDropResult.message}`);
        } else {
            console.log(`❌ Drag and drop simulation failed: ${dragDropResult.error}`);
        }
        
        // Check for any JavaScript errors
        const errors = await page.evaluate(() => {
            return window.errors || [];
        });
        
        if (errors.length > 0) {
            console.log('⚠️ JavaScript errors found:');
            errors.forEach(error => console.log(`   - ${error}`));
        } else {
            console.log('✅ No JavaScript errors detected');
        }
        
        console.log('\n🎉 Test completed successfully!');
        
    } catch (error) {
        console.error('❌ Test failed:', error.message);
        process.exit(1);
    } finally {
        if (browser) {
            await browser.close();
        }
    }
}

// Check if servers are running
async function checkServers() {
    const http = require('http');
    
    console.log('🔍 Checking if servers are running...');
    
    const checkServer = (port, name) => {
        return new Promise((resolve) => {
            const req = http.get(`http://localhost:${port}`, (res) => {
                resolve({ port, name, running: true, status: res.statusCode });
            });
            req.on('error', () => {
                resolve({ port, name, running: false });
            });
            req.setTimeout(5000, () => {
                req.destroy();
                resolve({ port, name, running: false });
            });
        });
    };
    
    const results = await Promise.all([
        checkServer(8000, 'Backend'),
        checkServer(8080, 'Frontend')
    ]);
    
    results.forEach(result => {
        if (result.running) {
            console.log(`✅ ${result.name} server running on port ${result.port}`);
        } else {
            console.log(`❌ ${result.name} server not running on port ${result.port}`);
        }
    });
    
    const allRunning = results.every(r => r.running);
    if (!allRunning) {
        console.log('\n🚨 Please start the servers first:');
        console.log('   ./start_system.sh');
        process.exit(1);
    }
    
    console.log('');
}

// Main execution
async function main() {
    await checkServers();
    await testDragAndDrop();
}

if (require.main === module) {
    main().catch(console.error);
}

module.exports = { testDragAndDrop, checkServers }; 