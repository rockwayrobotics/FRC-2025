
import logging
import time

import ntcore

def main():
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()

    count = 0
    cmdTopic = inst.getBooleanTopic('/Test/cmd')
    cmdSub = cmdTopic.subscribe(False)
    cmdPub = cmdTopic.publish()

    modeTopic = inst.getStringTopic('/Test/mode')
    modeSub = modeTopic.subscribe('None', ntcore.PubSubOptions(keepDuplicates=True))
    modePub = modeTopic.publish()

    countPub = inst.getIntegerTopic('/Test/count').publish()

    # set initial values so they show up in dashboard?
    cmdPub.set(False)
    modePub.set('Home')
    countPub.set(count)

    timeout = None
    while True:
        for evt in cmdSub.readQueue():
            count += 1
            countPub.set(count)
            logging.info('%d: cmd %s', count, evt)
            if evt.value:
                timeout = time.time() + 1.5
                logging.info('run command: %s', evt.value)
                modePub.set('Homing')

        if timeout is not None and time.time() > timeout:
            timeout = None
            cmdPub.set(False)
            modePub.set('Re-home')

        for evt in modeSub.readQueue():
            count += 1
            countPub.set(count)
            logging.info('%d: mode %s', count, evt)

        time.sleep(0.1)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG)

    main()

