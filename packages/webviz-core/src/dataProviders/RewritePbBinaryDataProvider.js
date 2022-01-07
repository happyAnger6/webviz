// @flow
//
//  Copyright (c) 2020-present, Cruise LLC
//
//  This source code is licensed under the Apache License, Version 2.0,
//  found in the LICENSE file in the root directory of this source tree.
//  You may not use this file except in compliance with the License.
import {groupBy} from 'lodash';
import {type Time, TimeUtil} from 'rosbag';

import BinaryMessageWriter from '../util/binaryObjects/binaryTranslation';
import pbMessageDefinitionsToParsed from './pbMessageDefinitionsToParsed';
import type {DataProviderDescriptor, DataProvider, GetDataProvider, ExtensionPoint, GetMessagesResult, GetMessagesTopics, InitializationResult,} from 'webviz-core/src/dataProviders/types';
import type {RosDatatypes} from 'webviz-core/src/types/RosDatatypes';
import {getObjects} from 'webviz-core/src/util/binaryObjects';
import {getContentBasedDatatypes} from 'webviz-core/src/util/datatypes';
import naturalSort from 'webviz-core/src/util/naturalSort';
import sendNotification from 'webviz-core/src/util/sendNotification';

export default class RewritePbBinaryDataProvider implements DataProvider {
  _provider: DataProvider;
  _extensionPoint: ExtensionPoint;
  _writer: BinaryMessageWriter;
  _datatypeByTopic: {[topic: string]: string};
  _datatypes: RosDatatypes;

  constructor(_: {||}, children: DataProviderDescriptor[],
      getDataProvider: GetDataProvider) {
    this._provider = getDataProvider(children[0]);
  }

  async initialize(extensionPoint: ExtensionPoint):
      Promise<InitializationResult> {
    this._extensionPoint = extensionPoint;
    const result = await this._provider.initialize(
        {...extensionPoint, progressCallback: () => {}});

    const {topics} = result;
    // If the child message definitions are not parsed, parse them here.
    const messageDefinitions = result.messageDefinitions.type === 'parsed' ?
        result.messageDefinitions :
        pbMessageDefinitionsToParsed(result.messageDefinitions, topics);

  //  this._writer = new BinaryMessageWriter();
   // await this._writer.initialize();

  /*  try {
      const datatypesByTopic = {};
      topics.forEach((topic) => {
        datatypesByTopic[topic.name] =
            topic.datatype;  // topic: /chassis_info_10ms,
                             // datatype:proto_msg/ChassisInfoRx10ms
      });
      const {fakeDatatypesByTopic, fakeDatatypes} = getContentBasedDatatypes(
          messageDefinitions.messageDefinitionsByTopic,
          messageDefinitions.parsedMessageDefinitionsByTopic, datatypesByTopic);
      this._writer.registerDefinitions(fakeDatatypes);
      this._datatypes = fakeDatatypes;
      this._datatypeByTopic = fakeDatatypesByTopic;
    } catch (err) {
      sendNotification(
          'Failed to register type definitions',
          err ? `${err.message} - ${err.stack}` : '<unknown error>', 'app',
          'error');
    }*/

    return {...result, messageDefinitions};
  }

  async getMessages(start: Time, end: Time, subscriptions: GetMessagesTopics):
      Promise<GetMessagesResult> {
    const {rosBinaryMessages} = await this._provider.getMessages(start, end, {
      rosBinaryMessages: subscriptions.bobjects,
    });

    return {
      bobjects: rosBinaryMessages.sort(
          (a, b) => TimeUtil.compare(a.receiveTime, b.receiveTime) ||
              naturalSort()(a.topic, b.topic)),
      rosBinaryMessages: undefined,
      parsedMessages: undefined,
    };
  }

  close(): Promise<void> {
    return this._provider.close();
  }
}
