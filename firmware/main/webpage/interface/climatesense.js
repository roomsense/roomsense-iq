document.addEventListener('DOMContentLoaded', function() {
  var chart1= {}, 
      chart2= {}, 
      chart3= {}, 
      chart4= {}, 
      chart5= {},
      chart6= {},
      chart7= {},
      chart8= {};

  let range = [0, 100];
  let labels_data = [];
  let temp_unit = 'c';

  Create_Waveform_Chart(chart1,"chart-1" ,"Temperature",range,false,  true, labels_data,0.4);
  Create_Waveform_Chart(chart2,"chart-2" ,"Humidity",  range,false, true, labels_data,0.4);
  Create_Waveform_Chart(chart3,"chart-3" ,"CO2",  range,false, true, labels_data,0.4);
  Create_Waveform_Chart(chart4,"chart-4" ,"VOC",  range,false, true, labels_data,0.4);
  Create_Waveform_Chart(chart5,"chart-5" ,"PM 10",  range,false, true, labels_data,0.4);
  Create_Waveform_Chart(chart6,"chart-6" ,"PM 2.5",  range,false, true, labels_data,0.4);
  Create_Waveform_Chart(chart7,"chart-7" ,"PM 1.0",  range,false, true, labels_data,0.4);
  Create_Waveform_Chart(chart8,"chart-8" ,"CO",  range,false, true, labels_data,0.4);
 
  start_websocket_client(1000);

  var sw1 = document.getElementById('switch-2'); 
  sw1.addEventListener('change', function() {
    if(temp_unit == 'c')
      temp_unit = 'f';
    else temp_unit = 'c';
    console.log(temp_unit);
  });

  const maxRetries = 2; // Maximum number of retry attempts
  let currentRetry = 0; // Current retry attempt
  var newData;

  function start_websocket_client(poll_interval){ 
    var gateway = `ws://${window.location.hostname}/ws`;
    socket = new WebSocket(gateway); 
    // Handle WebSocket connection opened event
    socket.onopen = function(event) {
        console.log('WebSocket connection opened');
        setInterval(function() {
            var message = 'data_request';
            socket.send(message);
            }, poll_interval
        ); 
        const data = { "value" : "0" , "name": "get_th_buf" }; 
        socket.send(JSON.stringify(data));  
  };
  
  socket.onclose = function(event) {
      if (currentRetry < maxRetries) {
      currentRetry++;
      console.log(`WebSocket connection closed. Retrying in 2 seconds... (Attempt ${currentRetry}/${maxRetries})`);
      setTimeout(start_websocket_client, 3500); // Retry after 2 seconds
      } 
      else {
          console.error('WebSocket connection failed after maximum retry attempts.');
      }
  };

  // Handle WebSocket message received event
  socket.onmessage = function(event) {
      console.log("received ",event.data);  
      newData = JSON.parse(event.data);
      if(1){
        if(newData.VOC[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart4.chart,newData.VOC, true,lbl);
        }

        if(newData.CO2[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart3.chart,newData.CO2, true,lbl);
        }

        if(newData.PM10[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart5.chart,newData.PM10, true,lbl);
        }
        if(newData.PM2_5[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart6.chart,newData.PM2_5, true,lbl);
        }
        if(newData.PM1_0[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart7.chart,newData.PM1_0, true,lbl);
        }

        if(newData.CO[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart8.chart,newData.CO, true,lbl);
        }

        if(newData.temperature[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart1.chart,newData.temperature, true,lbl);
        }
    
        if(newData.temperature_realtime[0] != "NULL"){   
          update_realtime_values("chart-1",'Temperature',newData.temperature_realtime[0]);  
        }
    
        //Humidity
        if(newData.humidity[0] != "NULL"){
          let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
          update_chartSP(chart2.chart,newData.humidity, true,lbl);
        }
        if(newData.humidity_realtime[0] != "NULL"){
          update_realtime_values("chart-2",'Humidity',newData.humidity_realtime[0]);
        }
    }
  };


  // Handle WebSocket connection error event
  socket.onerror = function(error) {
  console.error('WebSocket error:', error);
  };  
}

function conv_temp_c2f(temp){
  var fahrenheit = (temp * 9/5) + 32;
  return fahrenheit;
}
  
function Create_Waveform_Chart(waveform_obj, elementid, chart_title_name, range, xaxis_display, yaxis_display, labels_data, line_tension) {
  var ctx1 = document.getElementById(elementid).getContext('2d');

  waveform_obj.chart = new Chart(ctx1, {
    type: 'line',
    data: {
      labels: labels_data,
      datasets: [
        {
          data: [],
          borderWidth: 1,
          borderColor: 'orange',
          backgroundColor: 'rgba(0, 0, 0, 0)',
          pointRadius: 0,
          lineTension: line_tension,
          fill: false,
        },
        {
          data: [],
          borderWidth: 1,
          borderColor: '#03a9f4',
          backgroundColor: 'rgba(0, 0, 0, 0)',
          pointRadius: 0,
          lineTension: line_tension,
          fill: false,
        }
      ]
    },
    options: {
      layout: {
        padding: 40
      },
      width: 550,
      height: 350,
      responsive: false,
      backgroundColor: 'rgba(0, 0, 0, 1)',
      scales: {
        x: {
          type: 'category',
          offset: true,
          display: xaxis_display,
        },
        x2: {
          type: 'category',
          offset: true,
          display: false, // Set display to false to hide the x2-axis labels
        },
        y: {
          display: yaxis_display,
          grid: {
            color: 'rgba(255, 255, 255, 0.1)'
          },
          suggestedMin: range[0],
          suggestedMax: range[1]
        },

      },
      elements: {
        line: {
          backgroundColor: 'orange',
          borderColor: 'orange',
          tension: 0.3
        }
      },
      animation: {
        duration: 0
      },
      interaction: {
        mode: 'nearest',
        intersect: false
      },
      plugins: {
        title: {
          color: 'rgba(255, 255, 255, 1)',
          text: chart_title_name,
          display: true
        },
        legend: {
          display: false
        },
        streaming: {
          frameRate: 30,
          duration: 10000,
          delay: 0
        }
      }
    }
  });
}

function update_realtime_values(chart_id, type, value){
  const canvas = document.getElementById(chart_id);
  const context = canvas.getContext('2d');
  context.font = '24px Arial';
  context.fillStyle = 'white';
  let string = "";
  if(type == 'Temperature')  {
    if(temp_unit === 'f'){
      value = conv_temp_c2f(value);
      string = value + 'F';
    }
    else if(temp_unit === 'c'){
      string = value + 'C';
    }
    
  }
  else if(type == 'Humidity')     string = value + '%';
  else string = value;
  context.fillText(string, 45, 35);
}

function update_chartSP(ctx, newData, appendLabels, labelsData) {
  const maxPoints = 96;

  if (appendLabels) {
    // Append new labels to the existing labels
    ctx.data.labels.push(...labelsData);
    // Ensure there are no more than 16 labels
    if (ctx.data.labels.length > maxPoints) {
      ctx.data.labels = ctx.data.labels.slice(-maxPoints);
    }
  }

  // Append new data points to the existing dataset
  ctx.data.datasets[0].data.push(...newData);
  // Ensure there are no more than 16 data points
  if (ctx.data.datasets[0].data.length > maxPoints) {
    ctx.data.datasets[0].data = ctx.data.datasets[0].data.slice(-maxPoints);
  }

  // Update the chart
  ctx.update({
    duration: 14,
    easing: 'linear',
    preservation: true
  });
}

});


