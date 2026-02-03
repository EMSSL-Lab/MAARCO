use crate::protocol::PqtmOutput;

pub struct PqtmParser {
    incomplete_sentence: String,
}

impl PqtmParser {
    pub fn new() -> Self {
        PqtmParser {
            incomplete_sentence: String::new(),
        }
    }

    /// Parses incoming data for complete $PQTM* sentences.
    pub fn parse_data(&mut self, data: &str) -> Vec<PqtmOutput> {
        let mut complete_parsed_sentences: Vec<String> = Vec::new();
        let mut buffer = self.incomplete_sentence.clone() + data;

        // Loop to find complete sentences in the buffer. Break when the next sentence is
        // incomplete.
        loop {
            let start_index = match buffer.find("$PQTM") {
                Some(index) => index,
                None => {
                    // No start found, discard buffer
                    self.incomplete_sentence.clear();
                    break;
                }
            };

            // Find the end of the sentence:
            let end_index = match buffer[start_index..].find("\r\n") {
                Some(index) => start_index + index + 2, // Include \r\n
                None => {
                    // No end found, store incomplete sentence
                    self.incomplete_sentence = buffer[start_index..].to_string();
                    break;
                }
            };

            // Extract complete sentence
            let complete_sentence = &buffer[start_index..end_index];
            println!("Complete PQTM sentence: {}", complete_sentence);
            complete_parsed_sentences.push(complete_sentence.to_string());

            // Move the buffer forward:
            buffer = buffer[end_index..].to_string();
        }

        let mut pqtm_outputs: Vec<PqtmOutput> = Vec::new();
        // Process complete lines
        pqtm_outputs.extend(parse_pqtm_sentences(&mut complete_parsed_sentences));
        println!(
            "Length of complete_parsed_sentences: {}",
            complete_parsed_sentences.len()
        );

        pqtm_outputs
    }
}
