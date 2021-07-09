pipeline {
  agent any;

  environment {
    GITHUB_TOKEN = credentials('GitHubToken')
    GITHUB_URL_PREFIX = 'https://api.github.com/repos/Skobochka/icfpc-2021/statuses'
  }

  stages {
    stage('Build') {
      steps {
        timeout(time: 10, unit: 'MINUTES') {
          ansiColor('xterm') {
            sh "docker build -t icfpc2021-rust-image:${env.BUILD_TAG} ."
          }
        }
      }
    }

    stage('Build Unit Tests') {
      steps {
        ansiColor('xterm') {
          sh "docker run -t --rm -e RUST_BACKTRACE=1 --entrypoint ./build-test.sh icfpc2021-rust-image:${env.BUILD_TAG}"
        }
      }
    }

    stage('Run Unit Tests') {
      steps {
        ansiColor('xterm') {
          sh "docker run -t --rm -e RUST_BACKTRACE=1 --entrypoint ./test.sh icfpc2021-rust-image:${env.BUILD_TAG}"
        }
      }
    }

    // DISABLED until proper server found
    // stage('Smoke Test') {
    //   steps {
    //     sh "docker run -t --rm -e RUST_BACKTRACE=1 icfpc2021-rust-image:${env.BUILD_TAG} http://server:12345 2933935384595749692"
    //   }
    // }
  }
  post {
    cleanup {
      sh "docker rmi icfpc2021-rust-image:${env.BUILD_TAG} || true" // Do not signal error if no image found
    }
    always {
      script {
        def changeLog = "```";
        for (int i = 0; i < currentBuild.changeSets.size(); i++) {
          def entries = currentBuild.changeSets[i].items
          for (int j = 0; j < entries.length; j++) {
            def entry = entries[j]
            changeLog += "\n * \"${entry.msg}\" by ${entry.author}"
          }
        }
        changeLog += "```"

        telegramSend "*${currentBuild.currentResult}*  ${env.JOB_NAME}\nChanges:\n${changeLog}[Build log](${BUILD_URL}/console)"
      }
    }
    success {
      script {
        def actions = []
        for (int i = 0; i < currentBuild.changeSets.size(); i++) {
          def entries = currentBuild.changeSets[i].items
          for (int j = 0; j < entries.length; j++) {
            def entry = entries[j]
            actions << """
               set +x
               curl -s "$GITHUB_URL_PREFIX/${entry.commitId}" \
                 -H "Authorization: token $GITHUB_TOKEN" \
                 -H "Content-Type: application/json" \
                 -X POST \
                 -d \"{\\\"state\\\": \\\"success\\\", \\\"context\\\": \\\"continuous-integration/jenkins\\\", \\\"description\\\": \\\"Jenkins\\\", \\\"target_url\\\": \\\"$BUILD_URL/console\\\"}\" > /dev/null
              """
          }
        }

        for (int i = 0; i < actions.size(); i++) {
          sh actions[i]
        }
      }
    }
    failure {
      script {
        def actions = []
        for (int i = 0; i < currentBuild.changeSets.size(); i++) {
          def entries = currentBuild.changeSets[i].items
          for (int j = 0; j < entries.length; j++) {
            def entry = entries[j]
            actions << """
               set +x
               curl "$GITHUB_URL_PREFIX/${entry.commitId}" \
                 -H "Authorization: token $GITHUB_TOKEN" \
                 -H "Content-Type: application/json" \
                 -X POST \
                 -d \"{\\\"state\\\": \\\"failure\\\", \\\"context\\\": \\\"continuous-integration/jenkins\\\", \\\"description\\\": \\\"Jenkins\\\", \\\"target_url\\\": \\\"$BUILD_URL/console\\\"}\"
              """
          }
        }

        for (int i = 0; i < actions.size(); i++) {
          sh actions[i]
        }
      }
    }
  }
}