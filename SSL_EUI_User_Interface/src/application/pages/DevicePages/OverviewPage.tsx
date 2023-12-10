import {
  ChartContainer,
  LineChart,
  RealTimeDomain,
  TimeAxis,
  VerticalAxis,
} from '@electricui/components-desktop-charts'
import { Printer } from '@electricui/components-desktop'
import { Card } from '@blueprintjs/core'
import { Composition } from 'atomic-layout'
import { IntervalRequester } from '@electricui/components-core'
import { LightBulb } from '../../components/LightBulb'
import { useMessageDataSource } from '@electricui/core-timeseries'
import React from 'react'
import { RouteComponentProps } from '@reach/router'
import { Slider } from '@electricui/components-desktop-blueprint'
import { Environment,  GLTF, OrbitControls } from '@electricui/components-desktop-three'
 


const layoutDescription = `
  Chart Chart
  Light Slider
  Printer Printer
`

export const OverviewPage = (props: RouteComponentProps) => {
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

      <Composition areas={layoutDescription} gap={10} autoCols="1fr">
        {Areas => (
          <React.Fragment>
            <Areas.Chart>
              <Card>
                <div style={{ textAlign: 'center', marginBottom: '1em' }}>
                  <b>Quaternion Values</b>
                </div>
                <ChartContainer>
                  <LineChart dataSource={Quaternion1} />
                  <LineChart dataSource={Quaternion2} />
                  <LineChart dataSource={Quaternion3} />
                  <LineChart dataSource={Quaternion0} />
                  <RealTimeDomain window={20000} yMin={-1} yMax={1} delay={0} />
                  <TimeAxis label="Seconds" />
                  <VerticalAxis />
                  
                </ChartContainer>
              </Card>
            </Areas.Chart>
            <Printer accessor="Q0" precision={8} />
          </React.Fragment>
        )}
      </Composition>
    </React.Fragment>
  )
}
