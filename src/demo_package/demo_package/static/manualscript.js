document.addEventListener('DOMContentLoaded', function() {
    const onledElement = document.getElementById('manual-on-led');
    const offledElement = document.getElementById('manual-off-led');
    const onBtn = document.getElementById('manual-onBtn');
    const offBtn = document.getElementById('manual-offBtn');

    const autoonBtn = document.getElementById('auto-onBtn');
    const autooffBtn = document.getElementById('auto-offBtn');

    const autoonledElement = document.getElementById('auto-on-led');
    const autooffledElement = document.getElementById('auto-off-led');



    // Function to update the LED color
    function updateLEDColor(onColor,offColor,manualMode) {
        onledElement.style.backgroundColor = onColor;
        onledElement.style.boxShadow = onColor;

        offledElement.style.backgroundColor = offColor;
        offledElement.style.boxShadow = offColor;

        if (manualMode) {
            //If the program has started
            autoonBtn.setAttribute('disabled',true)
            autooffBtn.setAttribute('disabled',true)
            autoonledElement.style.backgroundColor = '#e4e4e4';
            autooffledElement.style.backgroundColor = '#DE7373'
        } else {
            //If the program has not started
            autoonBtn.removeAttribute('disabled')
            autooffBtn.removeAttribute('disabled')
        }

        // Send the color information to the Flask app using AJAX (fetch API)
        fetch('/manual_mode', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            manualMode: manualMode })
        })
        .then(response => response.json())
        .then(data => {
        // Handle the response from the Flask app if needed
        console.log(data);
        })
        .catch(error => {
        // Handle errors if any
        console.error('Error:', error);
        });
    }

    // Click event handler for the start button
    onBtn.addEventListener('click', function() {
        const startColor = '#49B265';
        const stopColor = '#e4e4e4';
        const flag = true
        updateLEDColor(startColor,stopColor,flag);
    });

    // Click event handler for the stop button
    offBtn.addEventListener('click', function() {
        const stopColor = '#DE7373';
        const startColor = '#e4e4e4';
        const flag = false
        updateLEDColor(startColor,stopColor,flag);
    });

    // Initialize the LED color (default to grey on page load)
    const startinitialColor = '#e4e4e4';
    const stopinitialColor = '#DE7373';
    const flag = false
    updateLEDColor(startinitialColor,stopinitialColor,flag);
    });