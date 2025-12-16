import { NitroModules } from 'react-native-nitro-modules'
import type { MAVLinkType } from './specs/MAVLink.nitro'

export const MAVLink = NitroModules.createHybridObject<MAVLinkType>('MAVLink')
