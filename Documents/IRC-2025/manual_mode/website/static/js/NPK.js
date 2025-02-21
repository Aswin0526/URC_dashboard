document.addEventListener('DOMContentLoaded', function () {
    const ctx = document.getElementById('NPK').getContext('2d');
    const npkData = {
        labels: [],
        datasets: [
            {
                label: 'Nitrogen (ppm)',
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(54, 162, 235, 0.2)',
                fill: true,
                data: []
            },
            {
                label: 'Phosphorus (ppm)',
                borderColor: 'rgba(200, 100, 100, 1)',
                backgroundColor: 'rgba(200, 100, 100, 0.2)',
                fill: true,
                data: []
            },
            {
                label: 'Potassium (ppm)',
                borderColor: 'rgba(255, 206, 86, 1)',
                backgroundColor: 'rgba(255, 206, 86, 0.2)',
                fill: true,
                data: []
            }
        ]
    };

    const npkConfig = {
        type: 'line',
        data: npkData,
        options: {
            responsive: true,
            scales: {
                x: {
                    title: { display: true, text: 'Time (HH:MM:SS)' }
                },
                y: {
                    title: { display: true, text: 'Concentration (ppm)' }
                }
            }
        }
    };

    const npkChart = new Chart(ctx, npkConfig);

    function formatTime(timestamp) {
        const date = new Date(timestamp);
        const minutes = String(date.getMinutes()).padStart(2, '0');
        const seconds = String(date.getSeconds()).padStart(2, '0');
        return `${minutes}:${seconds}`;
    }
    async function updateNpkData() {
        try {
            const response = await fetch('http://192.168.1.110:5000/science-subsystem/NPK');
            const data = await response.json()
            const nitrogen = data.nitrogen;      
            const phosphorus = data.phosphorus;    
            const potassium = data.potassium;     

            const currentTime = Date.now();
            npkData.labels.push(formatTime(currentTime));
            npkData.datasets[0].data.push(nitrogen);
            npkData.datasets[1].data.push(phosphorus);
            npkData.datasets[2].data.push(potassium);

            if (npkData.labels.length > 5) {
                npkData.labels.shift();
                npkData.datasets[0].data.shift();
                npkData.datasets[1].data.shift();
                npkData.datasets[2].data.shift();
            }

            npkChart.update();
        } catch (error) {
            console.error("Error updating NPK data:", error);
        }
    }

    setInterval(updateNpkData, 1000); // Update every 1 seconds
});
