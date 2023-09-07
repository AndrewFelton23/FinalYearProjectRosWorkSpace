document.addEventListener('DOMContentLoaded', function() {
    const StartledElement = document.getElementById('start-led');
    const StopledElement = document.getElementById('stop-led');

    const startBtn = document.getElementById('startBtn');
    const stopBtn = document.getElementById('stopBtn');

    const manualonBtn = document.getElementById('manual-onBtn');
    const manualoffBtn = document.getElementById('manual-offBtn');

    const manualonledElement = document.getElementById('manual-on-led');
    const manualoffledElement = document.getElementById('manual-off-led');

    const autoonBtn = document.getElementById('auto-onBtn');
    const autooffBtn = document.getElementById('auto-offBtn');

    const autoonledElement = document.getElementById('auto-on-led');
    const autooffledElement = document.getElementById('auto-off-led');

    // Function to update the LED color
    function updateLEDColor(startColor,stopColor,started) {
        StartledElement.style.backgroundColor = startColor;
        StartledElement.style.boxShadow = startColor;

        StopledElement.style.backgroundColor = stopColor;
        StopledElement.style.boxShadow = stopColor;

        if (started) {
            //If the program has started
            manualonBtn.removeAttribute('disabled');
            manualoffBtn.removeAttribute('disabled');
            autoonBtn.removeAttribute('disabled');
            autooffBtn.removeAttribute('disabled');
        } else {
            //If the program has not started
            manualonBtn.setAttribute('disabled',true)
            manualoffBtn.setAttribute('disabled',true)
            manualonledElement.style.backgroundColor = '#e4e4e4';
            manualoffledElement.style.backgroundColor = '#DE7373'
            autoonBtn.setAttribute('disabled',true);
            autooffBtn.setAttribute('disabled',true);
            autoonledElement.style.backgroundColor = '#e4e4e4';
            autooffledElement.style.backgroundColor = '#DE7373';
        }

        // Send the color information to the Flask app using AJAX (fetch API)
        fetch('/update_led_color', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            start: started })
        })
        .then(response => response.json())
        .then(data => {
        // Handle the response from the Flask app if needed
        console.log(data);
        })
        .catch(error => {
        // Handle errors if any
        console.error('Error:', error);
        StartledElement.style.backgroundColor ='#e4e4e4';
        StartledElement.style.boxShadow = '#e4e4e4';
        StopledElement.style.backgroundColor = '#DE7373';
        StopledElement.style.boxShadow = '#DE7373';

        startBtn.setAttribute('disabled',true)
        stopBtn.setAttribute('disabled',true)

        onBtn.setAttribute('disabled',true)
        offBtn.setAttribute('disabled',true)
        onledElement.style.backgroundColor = '#e4e4e4';
        offledElement.style.backgroundColor = '#DE7373';
        });
    }

    // Click event handler for the start button
    startBtn.addEventListener('click', function() {
        const startColor = '#49B265';
        const stopColor = '#e4e4e4';
        const flag = true
        updateLEDColor(startColor,stopColor,flag);
    });

    // Click event handler for the stop button
    stopBtn.addEventListener('click', function() {
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