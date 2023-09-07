// Function to continuously send a request to the Flask server and update the HTML content
function updateHTMLContent() {
    fetch('/get_coordinates') // Where '/get_coordinates' is the endpoint in the Flask app that provides the coordinates data
        .then(response => response.json())
        .then(data => {
            // Update the HTML content with the received data
            document.getElementById('joint1-response').textContent = data.joint1;
            document.getElementById('joint2-response').textContent = data.joint2;
            document.getElementById('joint3-response').textContent = data.joint3;
            document.getElementById('joint4-response').textContent = data.joint4;
            document.getElementById('gripper-response').textContent = data.gripper;

            // Call the function again after a certain time interval (e.g., 5 seconds)
            setTimeout(updateHTMLContent, 5000); // 5000 milliseconds = 5 seconds
        })
        .catch(error => {
            console.error('Error:', error);
            document.getElementById('joint1-response').textContent = "error recieving information";
            document.getElementById('joint2-response').textContent = "error recieving information";
            document.getElementById('joint3-response').textContent = "error recieving information";
            document.getElementById('joint4-response').textContent = "error recieving information";
            document.getElementById('gripper-response').textContent = "error recieving information";
        });
}

// Call the function to start the periodic updates
updateHTMLContent();