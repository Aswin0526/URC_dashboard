var data = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
];
var value = 0;
var config = {
    type: 'gauge',
    data: {
        labels: [
            '01', '02', '03', '04', '05', '06', 
            '07', '08', '09', '10', '11', '12', 
            '13', '14'
        ],
        datasets: [{
            data: data,
            value: value,
            backgroundColor: [
                'rgb(254 0 2)', 'rgb(255 84 0)', 'rgb(255 154 0)', 
                'rgb(255 203 1)', 'rgb(225 202 0)', 'rgb(149 213 28)', 
                'rgb(76 179 0)', 'rgb(1 154 10)', 'rgb(6 168 93)', 
                'rgb(1 169 143)', 'rgb(3 133 195)', 'rgb(2 76 199)', 
                'rgb(66 36 182)', 'rgb(79 17 178)'
            ],
            borderWidth: 2
        }]
    },
    options: {
        responsive: true,
        title: {
            display: false,
            text: 'pH Level',
            fontSize: 24,
            fontColor: 'black'
        },
        layout: {
            padding: {
                bottom: 30
            }
        },
        needle: {
            radiusPercentage: 2,
            widthPercentage: 3.2,
            lengthPercentage: 80,
            color: 'rgba(0, 0, 0, 1)'
        },
        valueLabel: {
            formatter: Math.round,
            fontSize: 18
        }
    }
};

document.addEventListener('DOMContentLoaded', function () {
    var ctx = document.getElementById('pH-chart').getContext('2d');
    window.myGauge = new Chart(ctx, config);

    async function fetchLivePHData() {
        try {
            const response = await fetch('http://192.168.1.110:5000/science-subsystem/pH-Level');
            const data = await response.json()
            console.log("pH_level " + data.pH_Level + "\n")
            return parseInt(data.pH_Level, 10)
        } catch (error) {
            console.error("Error fetching hydrogen concentration", error);
        }
    }

    setInterval(async function () {
        const newPHValue = await fetchLivePHData();
        console.log("newPHValue " + newPHValue)
        window.myGauge.config.data.datasets[0].value = newPHValue;
        window.myGauge.update();
    }, 1000);
});