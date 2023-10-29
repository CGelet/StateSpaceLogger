import { Composition } from 'atomic-layout'
import {
  IntervalRequester,
  useHardwareState,
} from '@electricui/components-core'
import { useMessageDataSource } from '@electricui/core-timeseries'
import React from 'react'
import { RouteComponentProps } from '@reach/router'
import { Printer } from '@electricui/components-desktop'
import { IconNames } from '@blueprintjs/icons'

import {
  Environment,
  ControlledGroup,
  GLTF,
  OrbitControls,
} from '@electricui/components-desktop-three'
import { RoundedBox } from '@react-three/drei'


import IMUModel from './imu.glb'

GLTF.preload(IMUModel)

export const IMUPose3D = () => {
    const Quaternion1 = useMessageDataSource('Q1')
    const Quaternion2 = useMessageDataSource('Q2')
    const Quaternion3 = useMessageDataSource('Q3')
    const Quaternion0 = useMessageDataSource('Q0')  
  return (
    
    
    <React.Fragment>
        <IntervalRequester interval={50} variables={['Q1']} />
      <IntervalRequester interval={50} variables={['Q2']} />
      <IntervalRequester interval={50} variables={['Q3']} />
      <IntervalRequester interval={50} variables={['Q0']} />
      <Environment
        camera={{
          fov: 50,
          position: [0, 0, -85],
        }}
        style={{ width: '100%', height: '25vh' }}
      >
        
        {/* <OrbitControls /> */}
        <ControlledGroup
          position={[0, 0, 0]}
          // positionAccessor={state => {
          //   return [state.acc[1] / 10, state.acc[2] / 10, state.acc[0] / 10]
          // }}
          quaternionAccessor={state => {
            state = [Quaternion1, Quaternion2, Quaternion3, Quaternion0]
            return [Quaternion1, Quaternion2, Quaternion3, Quaternion0]
          }}
        >
          <GLTF asset={IMUModel} />
        </ControlledGroup>
        <ambientLight intensity={0.1} />
        <hemisphereLight intensity={0.3} />
        <directionalLight intensity={1.0} />
      </Environment>
      <Printer accessor="Q1" precision={8} />
      <Printer accessor="Q2" precision={8} />
      <Printer accessor="Q3" precision={8} />
      <Printer accessor="Q0" precision={8} />
    </React.Fragment>
  )
}