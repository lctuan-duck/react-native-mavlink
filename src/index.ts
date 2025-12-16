import { NitroModules } from 'react-native-nitro-modules'
import type { MAVLink } from './specs/MAVLink.nitro'

export const HybridMAVLink = NitroModules.createHybridObject<MAVLink>('MAVLink')