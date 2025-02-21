document.addEventListener('DOMContentLoaded', () => {
    const mainCameraImg = document.getElementById('camera-main');
    const buttons = document.querySelectorAll('#slider button');

    buttons.forEach(button => {
        button.addEventListener('click', () => {
            const clickedCameraImg = button.querySelector('img');
            const tempSrc = mainCameraImg.src;
            mainCameraImg.src = clickedCameraImg.src;
            clickedCameraImg.src = tempSrc;
            console.log("mainCameraImg : " + clickedCameraImg.src);
        });
    });
});
