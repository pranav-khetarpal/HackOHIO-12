#include <WiFi.h>
#include <HTTPClient.h>

void setup() {
  Serial.begin(115200);

  // Replace with your WiFi credentials
  const char* ssid = "Your_WiFi_SSID";  // Add your WiFi SSID here
  const char* password = "Your_WiFi_Password";  // Add your WiFi Password here
  WiFi.begin(ssid, password);

  // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Set server URL (Replace with your FastAPI server's IP address and port)
    String serverUrl = "http://<your-pc-ip>:8080/warning";

    // Data to send to the server (as JSON)
    String data = "{\"obstacle\": \"detected\", \"distance\": 100}";

    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    // Send HTTP POST request
    int httpResponseCode = http.POST(data);

    // Check response
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server Response: " + response);
    } else {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }

    http.end(); // End connection
  }

  delay(5000); // Wait for 5 seconds before sending the next request
}
