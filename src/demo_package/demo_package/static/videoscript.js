        // Function to update the image every few seconds
        function updateImage() {
            var img = document.getElementById('video_feed');
            img.src = "{{ url_for('get_image') }}?" + new Date().getTime(); // Add a timestamp to prevent caching
            // Update the image every 5 seconds
            setTimeout(updateImage, 3000); // 5000 milliseconds = 5 seconds

        }

        // Call the updateImage function to start updating the image
        updateImage();