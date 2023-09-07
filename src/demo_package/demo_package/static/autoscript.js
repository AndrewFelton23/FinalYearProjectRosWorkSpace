document.addEventListener('DOMContentLoaded', function() {
    const autoonBtn = document.getElementById('auto-onBtn');
    const autooffBtn = document.getElementById('auto-offBtn');

    const autoonledElement = document.getElementById('auto-on-led');
    const autooffledElement = document.getElementById('auto-off-led');    

    const onledElement = document.getElementById('manual-on-led');
    const offledElement = document.getElementById('manual-off-led');

    const onBtn = document.getElementById('manual-onBtn');
    const offBtn = document.getElementById('manual-offBtn');




    // Function to update the LED color
    function updateLEDColor(onColor,offColor,autoMode) {
        autoonledElement.style.backgroundColor = onColor;
        autoonledElement.style.boxShadow = onColor;

        autooffledElement.style.backgroundColor = offColor;
        autooffledElement.style.boxShadow = offColor;

        if (autoMode) {
            //If the program has started
            onBtn.setAttribute('disabled',true)
            offBtn.setAttribute('disabled',true)
            onledElement.style.backgroundColor = '#e4e4e4';
            offledElement.style.backgroundColor = '#DE7373'
        } else {
            //If the program has not started
            onBtn.removeAttribute('disabled')
            offBtn.removeAttribute('disabled')
        }

        // Send the color information to the Flask app using AJAX (fetch API)
        fetch('/manual_mode', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            autoMode: autoMode })
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
    autoonBtn.addEventListener('click', function() {
        const startColor = '#49B265';
        const stopColor = '#e4e4e4';
        const flag = true
        updateLEDColor(startColor,stopColor,flag);
    });

    // Click event handler for the stop button
    autooffBtn.addEventListener('click', function() {
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