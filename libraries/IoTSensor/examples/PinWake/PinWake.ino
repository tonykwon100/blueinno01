/*
 Copyright (c) 2014 OpenSourceRF.com.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

int myPinCallback(uint32_t ulPin)
{
  digitalWrite(4, HIGH);
  delay(250);
  digitalWrite(4, LOW);
  return 0;  // don't exit RFduino_ULPDelay
}


void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  // processing handled by exiting RFduino_ULPDelay
  pinMode(0, INPUT);
  RFduino_pinWake(0, LOW);
  
  // processing handled by myPinCallback
 // pinMode(6, INPUT);
  //RFduino_pinWakeCallback(6, LOW, myPinCallback);
}

void loop() {
  //RFduino_ULPDelay(SECONDS(10));
  RFduino_ULPDelay(INFINITE);
  if (RFduino_pinWoke(0))
  {
    digitalWrite(3, HIGH);
    delay(5000);
    digitalWrite(3, LOW);                      
    
    RFduino_resetPinWake(0);
  }
  else
  {
    Serial.begin(9600);
    Serial.println("timed out");
    Serial.end();
    digitalWrite(2, HIGH);
    delay(250);
    digitalWrite(2, LOW);
  }
}
