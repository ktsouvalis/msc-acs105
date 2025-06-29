# Product Context: Fire Detection Robot

## Problem Statement
Building fires and industrial disasters create environments too dangerous for human first responders to enter safely. Traditional fire detection systems are stationary and limited in coverage area, especially in disaster scenarios where infrastructure may be damaged. There's a critical need for mobile, autonomous systems that can:
- Navigate through building debris and collapsed structures
- Detect fires and hazardous gases in inaccessible areas
- Provide real-time situational data to emergency responders
- Operate autonomously in environments unsafe for human personnel
- Cover large areas systematically without missing potential fire sources

## Solution Approach
The UVS Fire Detection Robot addresses these challenges by combining:
1. **Autonomous Mobile Platform**: Robot with Ackermann steering capable of navigating around building debris
2. **Systematic Navigation**: Right-hand rule algorithm ensures complete area coverage
3. **Multi-Modal Detection**: Gas sensors (MQ-2) for CO and LPG detection indicating fire presence
4. **Long-Range Telemetry**: LoRa communication for reliable data transmission up to 10km
5. **GPS Positioning**: Precise location data for emergency response coordination
6. **Extended Operation**: 4+ hour battery life for thorough exploration missions

## User Experience Goals

### Primary Users: Emergency Response Teams
- **Real-time Intelligence**: Receive continuous telemetry data including robot position, gas readings, and battery status
- **Fire Location Data**: Immediate alerts with GPS coordinates when fire/gas is detected above threshold levels
- **Safe Reconnaissance**: Deploy robot to investigate dangerous areas without risking human lives
- **Mission Monitoring**: Track robot progress and ensure systematic area coverage
- **Data Integration**: Compatible telemetry format for integration with emergency response systems

### Secondary Users: Search and Rescue Operations
- **Debris Navigation**: Robot can access areas blocked by collapsed structures
- **Survivor Detection Support**: Systematic exploration provides comprehensive area assessment
- **Environmental Assessment**: Continuous gas monitoring for responder safety planning

### Tertiary Users: Research/Academic Community
- **Technology Demonstration**: Showcase practical integration of robotics, IoT, and emergency response
- **Algorithm Validation**: Real-world testing of right-hand rule navigation in debris environments
- **Cost-Effective Platform**: €227 total cost makes technology accessible for research and education

## How It Should Work

### Mission Deployment
1. **Pre-Mission Setup**: Robot placed at entry point of target area
2. **Initialization**: GPS coordinates recorded as starting point, sensors calibrated
3. **Mission Start**: Robot begins systematic exploration using right-hand rule algorithm
4. **Continuous Monitoring**: Real-time telemetry transmission every second during operation

### Navigation Behavior
1. **Wall Following**: Maintain 20cm distance from right wall/obstacle
2. **Speed Adaptation**: 
   - Normal speed (200 PWM) when front is clear (>50cm)
   - Slow speed (50 PWM) when approaching obstacles (20-50cm)
   - Stop and turn when blocked (<20cm)
3. **Decision Making**:
   - Right opening available: Turn right to follow wall
   - Left opening only: Turn left to continue exploration
   - Both blocked: Execute turn-around maneuver

### Fire Detection Process
1. **Continuous Monitoring**: MQ-2 sensor constantly samples air for CO and LPG
2. **Threshold Detection**: Alert triggered when CO > 50ppm OR LPG > 2100ppm
3. **Emergency Response**: Immediate LoRa transmission with GPS coordinates and gas readings
4. **Mission Continuation**: Robot continues exploration unless commanded to return

### Data Transmission
- **Regular Telemetry**: Format: `lat, lon, LPG_ppm, CO_ppm, batt_voltage`
- **Transmission Rate**: 1Hz during normal operation
- **Emergency Alerts**: Immediate transmission when fire detected
- **Range**: Up to 10km line-of-sight with 433MHz LoRa

### Mission Completion
1. **Area Coverage**: Right-hand rule ensures complete exploration of accessible areas
2. **Return Navigation**: GPS-guided return to starting coordinates
3. **Data Recovery**: Complete mission data available for post-mission analysis

## Value Proposition

### Safety Benefits
- **Human Life Protection**: Eliminates need for personnel to enter dangerous environments
- **Early Detection**: Identifies fire sources before they spread beyond containment
- **Real-time Intelligence**: Provides situational awareness for informed response decisions
- **Systematic Coverage**: Ensures no accessible areas are missed during exploration

### Operational Advantages
- **Cost Effective**: €227 total cost significantly lower than commercial alternatives
- **Extended Range**: LoRa communication enables operation far from base stations
- **Autonomous Operation**: Requires minimal human intervention once deployed
- **Rapid Deployment**: Quick setup and immediate operational capability
- **Reliable Navigation**: Right-hand rule algorithm proven for systematic exploration

### Technical Innovation
- **Integrated Platform**: Combines navigation, detection, and communication in single system
- **Academic Accessibility**: Low cost enables educational use and research
- **Open Architecture**: Modular design allows for future enhancements and modifications
- **Real-world Application**: Addresses genuine emergency response challenges

## Target Environment Characteristics

### Physical Environment
- **Building Debris**: Collapsed structures, rubble, and irregular obstacles
- **Confined Spaces**: Narrow passages and confined areas inaccessible to humans
- **Variable Terrain**: Smooth floors to rough debris with height variations
- **Limited Visibility**: Smoke, dust, or darkness conditions

### Operational Constraints
- **Communication Range**: Must maintain LoRa link with base station
- **Battery Life**: 4+ hour missions with current power system
- **Environmental Hazards**: Potential presence of fires, toxic gases, unstable structures
- **Access Limitations**: Entry/exit points may be limited or hazardous

## Success Metrics

### Performance Indicators
- **Detection Accuracy**: >90% fire detection rate with <5% false positives
- **Coverage Efficiency**: Complete accessible area coverage within battery life
- **Communication Reliability**: >95% message delivery success rate
- **Navigation Success**: Ability to return to starting point after exploration
- **System Uptime**: >90% operational availability during missions

### Emergency Response Value
- **Response Time Improvement**: Faster initial assessment compared to human reconnaissance
- **Safety Enhancement**: Zero human exposure to hazardous environments during initial assessment
- **Data Quality**: Accurate GPS coordinates and quantitative gas measurements
- **Mission Reliability**: Consistent performance across different debris environments
