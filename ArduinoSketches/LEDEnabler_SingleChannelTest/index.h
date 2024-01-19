const char INDEX_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<style>
    * {
        font-family: Arial, Helvetica, sans-serif;
    }
    
    .main-wrapper {
        display: flex;
        flex-direction: column;
        width: 100%;
    }
    
    .info-section {
        display: flex;
        flex-direction: column;
        justify-content: center;
        max-width: 1000px;
        min-width: 500px;
        margin: 15px auto;
        background-color: #8BE08B;
        border-radius: 6px;
        border: 3px solid #4A8055;
        padding: 10px;
    }
    
    .title-section {
        text-align: center;
    }
    
    .navbar {
        text-align: center;
    }
    
    .error-section {
        text-align: center;
    }
    
    .bulb-section {
        display: flex;
        flex-direction: column;
        justify-content: center;
        max-width: 1000px;
        min-width: 500px;
        margin: 15px auto;
        background-color: #D8E8D8;
        border-radius: 6px;
        border: 3px solid #748074;
        padding: 10px;
    }
    
    .bulb-info-table {
        border: 1px solid #748074;
        border-radius 5px;
        border-collapse: collapse;
        width: 100%;
    }
    
    .bulb-info-table td {
        padding: 8px;
        border: 1px solid #748074;
    }
    
    .bulb-info-table th {
        border: 1px solid #748074;
        padding-top: 8px;
        padding-bottom: 8px;
        text-align: center;
        background-color: #04AA6D;
        color: white;
    }

</style>
<body>

	<div class="main-wrapper">
		<div class="info-section">
            <div class="title-section">
                <h2>Saab LED Enabler</h2>
                <h4>This page allows monitoring of the status of all bulbs.
                    <br>Click the "CONFIGURE" button below to set up your bulbs appropriately.</h4>
            </div>
            <div class="navbar">
                <a href="config.html" class="config">CONFIGURE</a>
            </div>
            <div class="error-section">
                <h4>Error Message: <span id="error-message">No error.</span></h4>
            </div>
        </div>
        
        <div class="bulb-section">
            <table class="bulb-info-table">
                <thead>
                    <tr>
                        <th>Bulb Name</th>
                        <th>Current State</th>
                        <th>Runtime</th>
                    </tr>
                </thead>
                <tbody>
                    <tr>
                        <td><span id="bulb1-name">Left Brake</td>
                        <td><span id="bulb1-state">Illuminated</td>
                        <td><span id="bulb1-runtime">1hr 5min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb2-name">Right Brake</td>
                        <td><span id="bulb2-state">Illuminated</td>
                        <td><span id="bulb2-runtime">1hr 5min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb3-name">Center Brake</td>
                        <td><span id="bulb3-state">Illuminated</td>
                        <td><span id="bulb3-runtime">1hr 5min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb4-name">Left Turn</td>
                        <td><span id="bulb4-state">Off</td>
                        <td><span id="bulb4-runtime">0hr 2min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb5-name">Right Turn</td>
                        <td><span id="bulb5-state">Off</td>
                        <td><span id="bulb5-runtime">0hr 2min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb6-name">Left Tail</td>
                        <td><span id="bulb6-state">Illuminated</td>
                        <td><span id="bulb6-runtime">1hr 5min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb7-name">Right Tail</td>
                        <td><span id="bulb7-state">Illuminated</td>
                        <td><span id="bulb7-runtime">1hr 5min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb8-name">Left Reverse</td>
                        <td><span id="bulb8-state">Illuminated</td>
                        <td><span id="bulb8-runtime">1hr 5min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb9-name">Right Reverse</td>
                        <td><span id="bulb9-state">Off</td>
                        <td><span id="bulb9-runtime">0hr 2min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb10-name">Left Fog</td>
                        <td><span id="bulb10-state">Off</td>
                        <td><span id="bulb10-runtime">0hr 2min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb11-name">Right Fog</td>
                        <td><span id="bulb11-state">Off</td>
                        <td><span id="bulb11-runtime">0hr 2min</span></td>
                    </tr>
                    <tr>
                        <td><span id="bulb12-name">Unused</td>
                        <td><span id="bulb12-state">N/A</td>
                        <td><span id="bulb12-runtime">N/A</span></td>
                    </tr>
                </tbody>
            </table>
        </div>

	<script>
		setInterval(function() {
			// Repeatedly call the refresh function
			refreshValues();
		}, 1000);
        
        setInterval(function() {
            // Refresh the runtimes every minute, not every second
            refreshRuntimes();
        }, 60000)

		function refreshValues() {
			var xhttp = new XMLHttpRequest();
			xhttp.onreadystatechange = function() {
				if (this.readyState == 4 && this.status == 200) {
					document.getElementById("ADCValue").innerHTML = this.responseText;
				}
			};
			xhttp.open("GET", "readADC", true);
			xhttp.send();
		}
        
        function refreshRuntimes() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("hi").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "readRuntimes", true);
            xhttp.send();
        }

	</script>

</body>
</html>
)=====";