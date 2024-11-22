/***************************************************
* Piezo Vibration Sensor
* ****************************************************
* This example The sensors detect vibration

* @author linfeng(490289303@qq.com)
* @version  V1.0
* @date  2016-2-26

* GNU Lesser General Public License.
* See <http://www.gnu.org/licenses/> for details.
* All above must be included in any redistribution
* ****************************************************/
#define sensorPin A0
void setup() {
  Serial.begin(115200);
}

void loop() {
  int x=analogRead(sensorPin);
  Serial.println(x);
  delay(50);
}


